Multi-Sensor Fusion
===================

NX-MIMOSA supports 6 sensor types with plug-in architecture.

Supported Sensors
-----------------

==================  =========================  ===============
Sensor Type         Measurement                Accuracy Gain
==================  =========================  ===============
Search Radar        Range + azimuth            Baseline
Fire Control Radar  Range + azimuth (precise)  +15%
Pulse-Doppler       Range + az + range-rate    +32%
EO/IR Camera        Bearing only               +4%
ESM/ELINT           Bearing only (passive)     +3%
ADS-B               Position + velocity        +64%
==================  =========================  ===============

Adding Sensors
--------------

Each sensor is a 2-line addition:

.. code-block:: python

   from nx_mimosa_fusion import (
       MultiSensorFusionEngine, SensorMeasurement,
       make_doppler_radar_sensor, make_eo_sensor, make_adsb_sensor
   )

   fusion = MultiSensorFusionEngine()
   fusion.add_sensor(make_doppler_radar_sensor("fc", r_std=10, az_std_deg=0.2, rdot_std=0.5))
   fusion.add_sensor(make_eo_sensor("flir", az_std_deg=0.05))
   fusion.add_sensor(make_adsb_sensor("adsb"))

   # In tracking loop:
   fusion.fuse(tracker, [SensorMeasurement("fc", doppler_data, time)])

Sensor Health Monitoring
------------------------

NX-MIMOSA continuously monitors each sensor's innovation statistics (NIS)
and flags degraded sensors:

.. code-block:: python

   health = fusion.get_health_report()
   for sensor_id, status in health.items():
       if status["degraded"]:
           print(f"WARNING: {sensor_id} NIS={status['nis_avg']:.1f}")
