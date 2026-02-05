// ═══════════════════════════════════════════════════════════════════════════════════════════════════
// NX-MIMOSA Multi-Domain Output Module
// ═══════════════════════════════════════════════════════════════════════════════════════════════════
//
// Runtime-switchable dual-domain track output:
//   - Civil ATC: ASTERIX Category 048 (EUROCONTROL standard, monoradar target reports)
//   - Military:  Link-16 J3.2 (MIL-STD-6016, air track message)
//
// Register Control:
//   REG_APPLICATION_MODE[0]:
//     0 = Civil ATC → ASTERIX Cat 048 packets via UDP/UART
//     1 = Military  → Link-16 J3.2 messages via JTIDS/MIDS interface
//
// Traceability:
//   [REQ-MULTI-DOMAIN-OUT-001] Multi-domain output formatting
//   [REQ-APPL-ATC-REAL-001]   Civil ATC compliance
//   [REQ-DEPLOY-7D-001]       Hardware deployment
//
// Author: Dr. Mladen Mešter / Nexellum d.o.o.
// Version: 4.0.0
// License: AGPL v3 / Commercial
// ═══════════════════════════════════════════════════════════════════════════════════════════════════

`timescale 1ns/1ps
`default_nettype none

// =============================================================================
// ASTERIX CAT 048 FORMATTER
// =============================================================================
// EUROCONTROL standard for monoradar target reports
// [REQ-MULTI-DOMAIN-OUT-001] [REQ-APPL-ATC-REAL-001]

module asterix_cat048_formatter #(
    parameter int MAX_TRACKS = 64,
    parameter int MAX_PKT_BYTES = 512,
    parameter logic [7:0] SAC = 8'h01,   // System Area Code (Croatia)
    parameter logic [7:0] SIC = 8'h42    // System Identification Code
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // Track input (from tracker/fusion)
    input  wire [31:0]              track_range_m   [MAX_TRACKS-1:0],  // Q16.16 range (m)
    input  wire [15:0]              track_azimuth    [MAX_TRACKS-1:0], // Q8.8 degrees
    input  wire [15:0]              track_altitude_ft[MAX_TRACKS-1:0], // Flight level * 100
    input  wire [11:0]              track_mode3a     [MAX_TRACKS-1:0], // Squawk code
    input  wire [23:0]              track_icao       [MAX_TRACKS-1:0], // ICAO 24-bit address
    input  wire [MAX_TRACKS-1:0]    track_valid,                       // Validity bitmap
    input  wire                     track_update,                      // New scan trigger

    // Time of day
    input  wire [23:0]              time_of_day_128s,                   // 1/128 second resolution

    // ASTERIX output (to UDP/UART transmitter)
    output logic [7:0]              asterix_data,
    output logic                    asterix_valid,
    output logic                    asterix_last,    // End of packet
    output logic [15:0]             asterix_pkt_len,
    output logic                    asterix_busy
);

    // Packet assembly state machine
    typedef enum logic [3:0] {
        ST_IDLE,
        ST_HEADER_CAT,
        ST_HEADER_LEN_H,
        ST_HEADER_LEN_L,
        ST_FSPEC,
        ST_I010_SAC,
        ST_I010_SIC,
        ST_I140_B2,
        ST_I140_B1,
        ST_I140_B0,
        ST_I040_RHO_H,
        ST_I040_RHO_L,
        ST_I040_THETA_H,
        ST_I040_THETA_L,
        ST_I070,
        ST_I090,
        ST_NEXT_TRACK,
        ST_DONE
    } state_t;

    state_t state;
    logic [$clog2(MAX_TRACKS)-1:0] track_idx;
    logic [15:0] byte_count;
    logic [15:0] total_len;

    // Pre-computed fields for current track
    logic [15:0] rho_256nm;    // Range in 1/256 NM
    logic [15:0] theta_65536;  // Azimuth in 360/65536 degrees

    // Range conversion: meters → 1/256 nautical miles
    // 1 NM = 1852 m, so rho = (range_m / 1852) * 256
    // Simplified: rho ≈ range_m * 256 / 1852 ≈ range_m * 9 / 65 (close approx)

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= ST_IDLE;
            track_idx     <= '0;
            byte_count    <= '0;
            asterix_data  <= '0;
            asterix_valid <= 1'b0;
            asterix_last  <= 1'b0;
            asterix_busy  <= 1'b0;
        end else begin
            asterix_valid <= 1'b0;
            asterix_last  <= 1'b0;

            case (state)
                ST_IDLE: begin
                    if (track_update) begin
                        state <= ST_HEADER_CAT;
                        track_idx <= '0;
                        byte_count <= '0;
                        asterix_busy <= 1'b1;

                        // Count valid tracks for length calculation
                        // Each record: 1(FSPEC) + 2(I010) + 3(I140) + 4(I040)
                        //            + 2(I070) + 2(I090) = 14 bytes
                        // Header: 3 bytes
                    end
                end

                ST_HEADER_CAT: begin
                    asterix_data  <= 8'd48;  // Category 048
                    asterix_valid <= 1'b1;
                    byte_count    <= byte_count + 1;
                    state         <= ST_HEADER_LEN_H;
                end

                ST_HEADER_LEN_H: begin
                    // Placeholder length (will be overwritten if streaming)
                    asterix_data  <= 8'h00;
                    asterix_valid <= 1'b1;
                    byte_count    <= byte_count + 1;
                    state         <= ST_HEADER_LEN_L;
                end

                ST_HEADER_LEN_L: begin
                    asterix_data  <= 8'h00;  // Placeholder
                    asterix_valid <= 1'b1;
                    byte_count    <= byte_count + 1;

                    // Find first valid track
                    state <= ST_NEXT_TRACK;
                end

                ST_NEXT_TRACK: begin
                    // Find next valid track
                    if (track_idx < MAX_TRACKS && track_valid[track_idx]) begin
                        // Pre-compute conversions
                        rho_256nm <= (track_range_m[track_idx][31:16] * 9) / 65;
                        theta_65536 <= (track_azimuth[track_idx] * 16'd182) >> 8; // deg→65536 scale

                        state <= ST_FSPEC;
                    end else if (track_idx < MAX_TRACKS) begin
                        track_idx <= track_idx + 1;
                    end else begin
                        state <= ST_DONE;
                    end
                end

                ST_FSPEC: begin
                    // FSPEC: I010, I140, I040, I070, I090, FX=0
                    asterix_data  <= 8'b11111000;
                    asterix_valid <= 1'b1;
                    byte_count    <= byte_count + 1;
                    state         <= ST_I010_SAC;
                end

                ST_I010_SAC: begin
                    asterix_data  <= SAC;
                    asterix_valid <= 1'b1;
                    byte_count    <= byte_count + 1;
                    state         <= ST_I010_SIC;
                end

                ST_I010_SIC: begin
                    asterix_data  <= SIC;
                    asterix_valid <= 1'b1;
                    byte_count    <= byte_count + 1;
                    state         <= ST_I140_B2;
                end

                ST_I140_B2: begin
                    asterix_data  <= time_of_day_128s[23:16];
                    asterix_valid <= 1'b1;
                    byte_count    <= byte_count + 1;
                    state         <= ST_I140_B1;
                end

                ST_I140_B1: begin
                    asterix_data  <= time_of_day_128s[15:8];
                    asterix_valid <= 1'b1;
                    byte_count    <= byte_count + 1;
                    state         <= ST_I140_B0;
                end

                ST_I140_B0: begin
                    asterix_data  <= time_of_day_128s[7:0];
                    asterix_valid <= 1'b1;
                    byte_count    <= byte_count + 1;
                    state         <= ST_I040_RHO_H;
                end

                ST_I040_RHO_H: begin
                    asterix_data  <= rho_256nm[15:8];
                    asterix_valid <= 1'b1;
                    byte_count    <= byte_count + 1;
                    state         <= ST_I040_RHO_L;
                end

                ST_I040_RHO_L: begin
                    asterix_data  <= rho_256nm[7:0];
                    asterix_valid <= 1'b1;
                    byte_count    <= byte_count + 1;
                    state         <= ST_I040_THETA_H;
                end

                ST_I040_THETA_H: begin
                    asterix_data  <= theta_65536[15:8];
                    asterix_valid <= 1'b1;
                    byte_count    <= byte_count + 1;
                    state         <= ST_I040_THETA_L;
                end

                ST_I040_THETA_L: begin
                    asterix_data  <= theta_65536[7:0];
                    asterix_valid <= 1'b1;
                    byte_count    <= byte_count + 1;
                    state         <= ST_I070;
                end

                ST_I070: begin
                    // Mode-3/A (12 bits) + flags
                    asterix_data  <= {4'b0000, track_mode3a[track_idx][11:8]};
                    asterix_valid <= 1'b1;
                    byte_count    <= byte_count + 1;
                    // Next byte
                    state         <= ST_I090;
                end

                ST_I090: begin
                    // Flight Level (1/4 FL)
                    asterix_data  <= track_altitude_ft[track_idx][15:8];
                    asterix_valid <= 1'b1;
                    byte_count    <= byte_count + 1;

                    // Advance to next track
                    track_idx <= track_idx + 1;
                    state     <= ST_NEXT_TRACK;
                end

                ST_DONE: begin
                    asterix_last   <= 1'b1;
                    asterix_valid  <= 1'b1;
                    asterix_data   <= 8'h00;
                    asterix_pkt_len <= byte_count;
                    asterix_busy   <= 1'b0;
                    state          <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule


// =============================================================================
// LINK-16 J3.2 AIR TRACK MESSAGE FORMATTER
// =============================================================================
// MIL-STD-6016 compliant track message
// [REQ-MULTI-DOMAIN-OUT-001]

module link16_j32_formatter #(
    parameter int MAX_TRACKS = 64
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // Track input (from tracker)
    input  wire signed [31:0]       track_lat_deg   [MAX_TRACKS-1:0],  // Q16.16
    input  wire signed [31:0]       track_lon_deg   [MAX_TRACKS-1:0],  // Q16.16
    input  wire [15:0]              track_alt_ft    [MAX_TRACKS-1:0],
    input  wire [15:0]              track_speed_kts [MAX_TRACKS-1:0],
    input  wire [15:0]              track_course_deg[MAX_TRACKS-1:0],  // Q8.8
    input  wire [1:0]               track_identity  [MAX_TRACKS-1:0],  // 0:unk 1:friend 2:hostile
    input  wire [2:0]               track_quality   [MAX_TRACKS-1:0],  // 0-7
    input  wire [13:0]              track_number    [MAX_TRACKS-1:0],
    input  wire [MAX_TRACKS-1:0]    track_valid,
    input  wire                     track_update,

    // Link-16 output (to JTIDS/MIDS encoder)
    output logic [74:0]             j32_word,        // 75-bit J-series word
    output logic                    j32_valid,
    output logic [13:0]             j32_track_num,
    output logic                    j32_busy
);

    typedef enum logic [1:0] {
        J_IDLE,
        J_FORMAT,
        J_OUTPUT,
        J_NEXT
    } jstate_t;

    jstate_t jstate;
    logic [$clog2(MAX_TRACKS)-1:0] tidx;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            jstate    <= J_IDLE;
            tidx      <= '0;
            j32_valid <= 1'b0;
            j32_busy  <= 1'b0;
            j32_word  <= '0;
        end else begin
            j32_valid <= 1'b0;

            case (jstate)
                J_IDLE: begin
                    if (track_update) begin
                        jstate   <= J_FORMAT;
                        tidx     <= '0;
                        j32_busy <= 1'b1;
                    end
                end

                J_FORMAT: begin
                    if (tidx < MAX_TRACKS && track_valid[tidx]) begin
                        // Pack J3.2 word
                        // [74:61] Track Number (14 bits)
                        // [60:38] Latitude in semicircles (23 bits)
                        // [37:15] Longitude in semicircles (23 bits)
                        // [14:5]  Speed (10 bits, 1 kt)
                        // [4:2]   Quality (3 bits)
                        // [1:0]   Identity (2 bits)

                        logic [22:0] lat_sc, lon_sc;
                        lat_sc = (track_lat_deg[tidx][31:16] * 23'd46603) >> 15;  // deg→semicircles
                        lon_sc = (track_lon_deg[tidx][31:16] * 23'd23302) >> 15;

                        j32_word <= {
                            track_number[tidx],                           // [74:61] TN
                            lat_sc,                                        // [60:38] Lat
                            lon_sc,                                        // [37:15] Lon
                            track_speed_kts[tidx][9:0],                   // [14:5]  Speed
                            track_quality[tidx],                           // [4:2]   Quality
                            track_identity[tidx]                           // [1:0]   Identity
                        };

                        j32_track_num <= track_number[tidx];
                        jstate <= J_OUTPUT;
                    end else if (tidx < MAX_TRACKS) begin
                        tidx <= tidx + 1;
                    end else begin
                        j32_busy <= 1'b0;
                        jstate   <= J_IDLE;
                    end
                end

                J_OUTPUT: begin
                    j32_valid <= 1'b1;
                    jstate    <= J_NEXT;
                end

                J_NEXT: begin
                    tidx   <= tidx + 1;
                    jstate <= J_FORMAT;
                end

                default: jstate <= J_IDLE;
            endcase
        end
    end

endmodule


// =============================================================================
// MULTI-DOMAIN OUTPUT TOP MODULE
// =============================================================================
// [REQ-MULTI-DOMAIN-OUT-001]

module multi_domain_output #(
    parameter int MAX_TRACKS = 64
)(
    input  wire         clk,
    input  wire         rst_n,

    // Mode register (AXI-Lite)
    input  wire         reg_app_mode,    // 0: Civil (ASTERIX), 1: Military (Link-16)

    // Common track input
    input  wire [31:0]  track_range_m    [MAX_TRACKS-1:0],
    input  wire [15:0]  track_azimuth    [MAX_TRACKS-1:0],
    input  wire [15:0]  track_altitude_ft[MAX_TRACKS-1:0],
    input  wire signed [31:0] track_lat  [MAX_TRACKS-1:0],
    input  wire signed [31:0] track_lon  [MAX_TRACKS-1:0],
    input  wire [15:0]  track_speed_kts  [MAX_TRACKS-1:0],
    input  wire [15:0]  track_course     [MAX_TRACKS-1:0],
    input  wire [11:0]  track_mode3a     [MAX_TRACKS-1:0],
    input  wire [23:0]  track_icao       [MAX_TRACKS-1:0],
    input  wire [13:0]  track_number     [MAX_TRACKS-1:0],
    input  wire [1:0]   track_identity   [MAX_TRACKS-1:0],
    input  wire [2:0]   track_quality    [MAX_TRACKS-1:0],
    input  wire [MAX_TRACKS-1:0] track_valid,
    input  wire         track_update,

    // Time
    input  wire [23:0]  time_of_day_128s,

    // ASTERIX output (civil)
    output wire [7:0]   asterix_data,
    output wire         asterix_valid,
    output wire         asterix_last,

    // Link-16 output (military)
    output wire [74:0]  link16_word,
    output wire         link16_valid,
    output wire [13:0]  link16_tn,

    // Status
    output wire         output_busy
);

    wire asterix_busy_w, link16_busy_w;

    // ASTERIX formatter (always instantiated, gated by mode)
    asterix_cat048_formatter #(.MAX_TRACKS(MAX_TRACKS)) u_asterix (
        .clk(clk), .rst_n(rst_n),
        .track_range_m(track_range_m),
        .track_azimuth(track_azimuth),
        .track_altitude_ft(track_altitude_ft),
        .track_mode3a(track_mode3a),
        .track_icao(track_icao),
        .track_valid(track_valid),
        .track_update(track_update & ~reg_app_mode),  // Only trigger in civil mode
        .time_of_day_128s(time_of_day_128s),
        .asterix_data(asterix_data),
        .asterix_valid(asterix_valid),
        .asterix_last(asterix_last),
        .asterix_pkt_len(),
        .asterix_busy(asterix_busy_w)
    );

    // Link-16 formatter
    link16_j32_formatter #(.MAX_TRACKS(MAX_TRACKS)) u_link16 (
        .clk(clk), .rst_n(rst_n),
        .track_lat_deg(track_lat),
        .track_lon_deg(track_lon),
        .track_alt_ft(track_altitude_ft),
        .track_speed_kts(track_speed_kts),
        .track_course_deg(track_course),
        .track_identity(track_identity),
        .track_quality(track_quality),
        .track_number(track_number),
        .track_valid(track_valid),
        .track_update(track_update & reg_app_mode),  // Only trigger in military mode
        .j32_word(link16_word),
        .j32_valid(link16_valid),
        .j32_track_num(link16_tn),
        .j32_busy(link16_busy_w)
    );

    assign output_busy = asterix_busy_w | link16_busy_w;

endmodule

`default_nettype wire
