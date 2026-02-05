#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════════════════════════════════
NX-MIMOSA GAN Adversarial Jammer Generator
═══════════════════════════════════════════════════════════════════════════════════════════════════════

Generative Adversarial Network for synthesizing "zero-day" jammer waveforms.
Used for adversarial training of RL agents to ensure robustness against unknown threats.

Architecture:
  - Generator: Latent noise → Complex I/Q jammer waveform
  - Discriminator: Real/Fake jammer classification
  - Conditioned on jammer type (RGPO, VGPO, Barrage, Spot)

Traceability:
  [REQ-RL-GAN-001] GAN adversarial jammer synthesis
  [REQ-ECCM-ADV] Zero-day jammer robustness

Author: Dr. Mladen Mešter / Nexellum d.o.o.
Version: 1.2.0
License: AGPL v3 / Commercial
═══════════════════════════════════════════════════════════════════════════════════════════════════════
"""

import numpy as np
from dataclasses import dataclass
from typing import Tuple, Optional, List
from enum import IntEnum

# Optional PyTorch imports
try:
    import torch
    import torch.nn as nn
    import torch.optim as optim
    from torch.utils.data import DataLoader, TensorDataset
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False


# =============================================================================
# CONFIGURATION
# =============================================================================

@dataclass
class GANConfig:
    """GAN hyperparameters."""
    # Architecture
    latent_dim: int = 128
    signal_length: int = 1024
    n_classes: int = 4  # RGPO, VGPO, Barrage, Spot
    
    # Generator
    gen_hidden_dims: Tuple[int, ...] = (256, 512, 1024)
    
    # Discriminator
    disc_hidden_dims: Tuple[int, ...] = (512, 256, 128)
    
    # Training
    batch_size: int = 64
    n_epochs: int = 100
    lr_gen: float = 2e-4
    lr_disc: float = 2e-4
    beta1: float = 0.5
    beta2: float = 0.999
    
    # Label smoothing for stability
    real_label_smooth: float = 0.9
    fake_label_smooth: float = 0.1


class JammerTypeGAN(IntEnum):
    """Jammer types for conditional GAN."""
    RGPO = 0
    VGPO = 1
    BARRAGE = 2
    SPOT = 3


# =============================================================================
# GENERATOR NETWORK
# =============================================================================

if TORCH_AVAILABLE:
    class Generator(nn.Module):
        """
        Conditional Generator for jammer waveforms.
        
        Input: (latent_noise, class_embedding)
        Output: Complex I/Q signal (2 x signal_length)
        """
        
        def __init__(self, config: GANConfig):
            super().__init__()
            self.config = config
            
            # Class embedding
            self.class_embed = nn.Embedding(config.n_classes, config.latent_dim)
            
            # Main network
            input_dim = config.latent_dim * 2  # noise + class embedding
            
            layers = []
            prev_dim = input_dim
            
            for hidden_dim in config.gen_hidden_dims:
                layers.extend([
                    nn.Linear(prev_dim, hidden_dim),
                    nn.BatchNorm1d(hidden_dim),
                    nn.LeakyReLU(0.2, inplace=True)
                ])
                prev_dim = hidden_dim
            
            # Output: I and Q channels
            layers.append(nn.Linear(prev_dim, config.signal_length * 2))
            layers.append(nn.Tanh())  # Normalized output
            
            self.net = nn.Sequential(*layers)
        
        def forward(self, noise: torch.Tensor, labels: torch.Tensor) -> torch.Tensor:
            """
            Generate jammer waveform.
            
            Args:
                noise: (batch, latent_dim)
                labels: (batch,) class indices
            
            Returns:
                (batch, 2, signal_length) complex signal as [I, Q]
            """
            class_emb = self.class_embed(labels)
            x = torch.cat([noise, class_emb], dim=1)
            out = self.net(x)
            return out.view(-1, 2, self.config.signal_length)


    class Discriminator(nn.Module):
        """
        Conditional Discriminator for real/fake classification.
        
        Input: Complex I/Q signal + class label
        Output: Real/Fake probability
        """
        
        def __init__(self, config: GANConfig):
            super().__init__()
            self.config = config
            
            # Class embedding (projected to signal space)
            self.class_embed = nn.Embedding(config.n_classes, config.signal_length)
            
            # Main network
            input_dim = config.signal_length * 3  # I + Q + class
            
            layers = []
            prev_dim = input_dim
            
            for hidden_dim in config.disc_hidden_dims:
                layers.extend([
                    nn.Linear(prev_dim, hidden_dim),
                    nn.LayerNorm(hidden_dim),
                    nn.LeakyReLU(0.2, inplace=True),
                    nn.Dropout(0.3)
                ])
                prev_dim = hidden_dim
            
            # Output: single logit
            layers.append(nn.Linear(prev_dim, 1))
            
            self.net = nn.Sequential(*layers)
        
        def forward(self, signal: torch.Tensor, labels: torch.Tensor) -> torch.Tensor:
            """
            Discriminate real vs fake.
            
            Args:
                signal: (batch, 2, signal_length) complex signal
                labels: (batch,) class indices
            
            Returns:
                (batch, 1) real/fake logits
            """
            batch_size = signal.shape[0]
            
            # Flatten I/Q channels
            signal_flat = signal.view(batch_size, -1)
            
            # Class embedding
            class_emb = self.class_embed(labels)
            
            # Concatenate
            x = torch.cat([signal_flat, class_emb], dim=1)
            
            return self.net(x)


# =============================================================================
# REAL JAMMER DATA GENERATOR
# =============================================================================

class RealJammerGenerator:
    """
    Generate "real" jammer samples for GAN training.
    
    Physics-based simulation of known jammer types.
    """
    
    def __init__(self, signal_length: int = 1024, fs: float = 100e6):
        self.signal_length = signal_length
        self.fs = fs
        self.t = np.arange(signal_length) / fs
    
    def generate_rgpo(self, jnr_db: float = 40, delay_rate: float = 1e-6) -> np.ndarray:
        """Generate RGPO jammer signal."""
        amplitude = 10 ** (jnr_db / 20)
        
        # Pulsed with delay modulation
        pulse_width = int(0.01 * self.signal_length)
        signal = np.zeros(self.signal_length, dtype=complex)
        
        n_pulses = 5
        for p in range(n_pulses):
            delay = int(p * delay_rate * self.fs * 10)
            start = int(p * self.signal_length / n_pulses) + delay
            end = min(start + pulse_width, self.signal_length)
            
            if start < self.signal_length:
                phase = np.random.uniform(0, 2 * np.pi)
                signal[start:end] = amplitude * np.exp(1j * phase)
        
        # Add noise
        signal += (np.random.randn(self.signal_length) + 
                  1j * np.random.randn(self.signal_length)) * 0.1
        
        return signal
    
    def generate_vgpo(self, jnr_db: float = 35, doppler_rate: float = 5000) -> np.ndarray:
        """Generate VGPO jammer signal."""
        amplitude = 10 ** (jnr_db / 20)
        
        # Progressive Doppler shift
        doppler = np.linspace(0, doppler_rate, self.signal_length)
        phase = 2 * np.pi * np.cumsum(doppler / self.fs)
        
        # Pulsed envelope
        envelope = np.zeros(self.signal_length)
        pulse_width = int(0.02 * self.signal_length)
        
        for p in range(4):
            start = int(p * self.signal_length / 4)
            end = min(start + pulse_width, self.signal_length)
            envelope[start:end] = 1.0
        
        signal = amplitude * envelope * np.exp(1j * phase)
        signal += (np.random.randn(self.signal_length) + 
                  1j * np.random.randn(self.signal_length)) * 0.1
        
        return signal
    
    def generate_barrage(self, jnr_db: float = 45) -> np.ndarray:
        """Generate barrage (wideband) jammer."""
        amplitude = 10 ** (jnr_db / 20)
        
        # Wideband noise
        signal = amplitude * (np.random.randn(self.signal_length) + 
                             1j * np.random.randn(self.signal_length))
        
        return signal
    
    def generate_spot(self, jnr_db: float = 40, freq_offset: float = 1e6) -> np.ndarray:
        """Generate spot (narrowband CW) jammer."""
        amplitude = 10 ** (jnr_db / 20)
        
        # CW tone
        signal = amplitude * np.exp(2j * np.pi * freq_offset * self.t)
        
        # Add slight AM modulation
        signal *= (1 + 0.1 * np.sin(2 * np.pi * 50 * self.t))
        
        # Add noise floor
        signal += (np.random.randn(self.signal_length) + 
                  1j * np.random.randn(self.signal_length)) * 0.05
        
        return signal
    
    def generate_batch(self, batch_size: int, 
                      jammer_type: Optional[int] = None) -> Tuple[np.ndarray, np.ndarray]:
        """
        Generate batch of jammer samples.
        
        Returns:
            signals: (batch, 2, signal_length) as [I, Q]
            labels: (batch,) class indices
        """
        signals = []
        labels = []
        
        generators = [
            self.generate_rgpo,
            self.generate_vgpo,
            self.generate_barrage,
            self.generate_spot
        ]
        
        for _ in range(batch_size):
            if jammer_type is None:
                label = np.random.randint(0, 4)
            else:
                label = jammer_type
            
            # Random JNR variation
            jnr = np.random.uniform(30, 50)
            
            signal = generators[label](jnr_db=jnr)
            
            # Convert to I/Q channels
            iq = np.stack([signal.real, signal.imag], axis=0)
            
            # Normalize
            iq = iq / (np.abs(iq).max() + 1e-8)
            
            signals.append(iq)
            labels.append(label)
        
        return np.array(signals, dtype=np.float32), np.array(labels, dtype=np.int64)


# =============================================================================
# GAN TRAINER
# =============================================================================

class AdversarialJammerGAN:
    """
    Complete GAN training system for adversarial jammer generation.
    
    [REQ-RL-GAN-001] GAN adversarial jammer synthesis
    """
    
    def __init__(self, config: GANConfig):
        self.config = config
        
        if not TORCH_AVAILABLE:
            raise ImportError("PyTorch required for GAN training")
        
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        
        # Networks
        self.generator = Generator(config).to(self.device)
        self.discriminator = Discriminator(config).to(self.device)
        
        # Optimizers
        self.opt_gen = optim.Adam(
            self.generator.parameters(),
            lr=config.lr_gen,
            betas=(config.beta1, config.beta2)
        )
        self.opt_disc = optim.Adam(
            self.discriminator.parameters(),
            lr=config.lr_disc,
            betas=(config.beta1, config.beta2)
        )
        
        # Loss
        self.criterion = nn.BCEWithLogitsLoss()
        
        # Data generator
        self.real_generator = RealJammerGenerator(config.signal_length)
        
        # Training stats
        self.gen_losses = []
        self.disc_losses = []
    
    def train_epoch(self) -> Tuple[float, float]:
        """Train for one epoch."""
        self.generator.train()
        self.discriminator.train()
        
        n_batches = 100
        total_gen_loss = 0
        total_disc_loss = 0
        
        for _ in range(n_batches):
            batch_size = self.config.batch_size
            
            # ═══════════════════════════════════════════════════════════════
            # Train Discriminator
            # ═══════════════════════════════════════════════════════════════
            self.opt_disc.zero_grad()
            
            # Real samples
            real_signals, labels = self.real_generator.generate_batch(batch_size)
            real_signals = torch.FloatTensor(real_signals).to(self.device)
            labels = torch.LongTensor(labels).to(self.device)
            
            real_labels = torch.full((batch_size, 1), self.config.real_label_smooth, 
                                    device=self.device)
            
            real_output = self.discriminator(real_signals, labels)
            loss_real = self.criterion(real_output, real_labels)
            
            # Fake samples
            noise = torch.randn(batch_size, self.config.latent_dim, device=self.device)
            fake_class = torch.randint(0, self.config.n_classes, (batch_size,), 
                                      device=self.device)
            
            fake_signals = self.generator(noise, fake_class)
            fake_labels = torch.full((batch_size, 1), self.config.fake_label_smooth,
                                    device=self.device)
            
            fake_output = self.discriminator(fake_signals.detach(), fake_class)
            loss_fake = self.criterion(fake_output, fake_labels)
            
            loss_disc = loss_real + loss_fake
            loss_disc.backward()
            self.opt_disc.step()
            
            # ═══════════════════════════════════════════════════════════════
            # Train Generator
            # ═══════════════════════════════════════════════════════════════
            self.opt_gen.zero_grad()
            
            noise = torch.randn(batch_size, self.config.latent_dim, device=self.device)
            fake_class = torch.randint(0, self.config.n_classes, (batch_size,),
                                      device=self.device)
            
            fake_signals = self.generator(noise, fake_class)
            
            # Generator wants discriminator to output 1 (real)
            gen_labels = torch.ones(batch_size, 1, device=self.device)
            gen_output = self.discriminator(fake_signals, fake_class)
            
            loss_gen = self.criterion(gen_output, gen_labels)
            loss_gen.backward()
            self.opt_gen.step()
            
            total_gen_loss += loss_gen.item()
            total_disc_loss += loss_disc.item()
        
        avg_gen = total_gen_loss / n_batches
        avg_disc = total_disc_loss / n_batches
        
        self.gen_losses.append(avg_gen)
        self.disc_losses.append(avg_disc)
        
        return avg_gen, avg_disc
    
    def train(self, verbose: bool = True) -> dict:
        """Full training loop."""
        stats = {
            'gen_losses': [],
            'disc_losses': []
        }
        
        for epoch in range(self.config.n_epochs):
            gen_loss, disc_loss = self.train_epoch()
            
            stats['gen_losses'].append(gen_loss)
            stats['disc_losses'].append(disc_loss)
            
            if verbose and (epoch + 1) % 10 == 0:
                print(f"Epoch {epoch+1:3d} | Gen Loss: {gen_loss:.4f} | "
                      f"Disc Loss: {disc_loss:.4f}")
        
        return stats
    
    def generate_adversarial_samples(self, n_samples: int, 
                                    jammer_type: Optional[int] = None) -> np.ndarray:
        """
        Generate adversarial jammer samples for RL training.
        
        Args:
            n_samples: Number of samples
            jammer_type: Specific type or None for random
        
        Returns:
            Complex signals (n_samples, signal_length)
        """
        self.generator.eval()
        
        with torch.no_grad():
            noise = torch.randn(n_samples, self.config.latent_dim, device=self.device)
            
            if jammer_type is not None:
                labels = torch.full((n_samples,), jammer_type, 
                                   dtype=torch.long, device=self.device)
            else:
                labels = torch.randint(0, self.config.n_classes, (n_samples,),
                                      device=self.device)
            
            fake_signals = self.generator(noise, labels)
            
            # Convert to complex
            iq = fake_signals.cpu().numpy()
            complex_signals = iq[:, 0, :] + 1j * iq[:, 1, :]
        
        return complex_signals
    
    def save(self, path: str):
        """Save GAN state."""
        torch.save({
            'generator': self.generator.state_dict(),
            'discriminator': self.discriminator.state_dict(),
            'config': self.config,
            'gen_losses': self.gen_losses,
            'disc_losses': self.disc_losses
        }, path)
    
    def load(self, path: str):
        """Load GAN state."""
        checkpoint = torch.load(path)
        self.generator.load_state_dict(checkpoint['generator'])
        self.discriminator.load_state_dict(checkpoint['discriminator'])
        self.gen_losses = checkpoint.get('gen_losses', [])
        self.disc_losses = checkpoint.get('disc_losses', [])


# =============================================================================
# NUMPY-ONLY FALLBACK (for environments without PyTorch)
# =============================================================================

class SimpleAdversarialGenerator:
    """
    Simple adversarial jammer generator without deep learning.
    
    Uses parametric models with random perturbations for robustness testing.
    """
    
    def __init__(self, signal_length: int = 1024):
        self.signal_length = signal_length
        self.real_gen = RealJammerGenerator(signal_length)
    
    def generate_adversarial(self, base_type: int, n_samples: int = 1) -> np.ndarray:
        """
        Generate adversarial variants of known jammer types.
        
        Applies random perturbations to create "zero-day" variants.
        """
        signals = []
        
        for _ in range(n_samples):
            # Generate base signal
            jnr = np.random.uniform(25, 55)
            
            if base_type == 0:  # RGPO variant
                signal = self.real_gen.generate_rgpo(jnr)
                # Adversarial: add random phase jumps
                jump_points = np.random.randint(0, self.signal_length, 5)
                for jp in jump_points:
                    signal[jp:] *= np.exp(1j * np.random.uniform(0, np.pi))
            
            elif base_type == 1:  # VGPO variant
                signal = self.real_gen.generate_vgpo(jnr)
                # Adversarial: non-linear Doppler
                t = np.arange(self.signal_length)
                nonlinear_doppler = np.exp(1j * 0.01 * t**1.5)
                signal *= nonlinear_doppler
            
            elif base_type == 2:  # Barrage variant
                signal = self.real_gen.generate_barrage(jnr)
                # Adversarial: add sparse impulses
                impulses = np.random.randint(0, self.signal_length, 10)
                signal[impulses] *= 5
            
            else:  # Spot variant
                signal = self.real_gen.generate_spot(jnr)
                # Adversarial: frequency hopping within narrowband
                hop_freqs = np.random.uniform(-5e5, 5e5, 5)
                segment_len = self.signal_length // 5
                for i, freq in enumerate(hop_freqs):
                    start = i * segment_len
                    end = (i + 1) * segment_len
                    t = np.arange(end - start) / 100e6
                    signal[start:end] *= np.exp(2j * np.pi * freq * t)
            
            signals.append(signal)
        
        return np.array(signals)
    
    def compute_snr_degradation(self, target_signal: np.ndarray, 
                               jammer_signal: np.ndarray) -> float:
        """Compute SNR degradation caused by jammer."""
        target_power = np.mean(np.abs(target_signal) ** 2)
        jammer_power = np.mean(np.abs(jammer_signal) ** 2)
        
        # Original SNR (assuming noise floor)
        noise_power = 0.01
        original_snr = 10 * np.log10(target_power / noise_power)
        
        # SNR with jammer
        degraded_snr = 10 * np.log10(target_power / (noise_power + jammer_power))
        
        return original_snr - degraded_snr


# =============================================================================
# MAIN
# =============================================================================

def main():
    """Train GAN and demonstrate adversarial generation."""
    print("═" * 70)
    print("NX-MIMOSA GAN Adversarial Jammer Training")
    print("═" * 70)
    
    config = GANConfig(
        latent_dim=128,
        signal_length=512,
        n_epochs=50,
        batch_size=32
    )
    
    if TORCH_AVAILABLE:
        print("\nTraining conditional GAN...")
        gan = AdversarialJammerGAN(config)
        stats = gan.train(verbose=True)
        
        # Generate samples
        print("\nGenerating adversarial samples...")
        for jtype in range(4):
            samples = gan.generate_adversarial_samples(10, jammer_type=jtype)
            avg_power = np.mean(np.abs(samples) ** 2)
            print(f"  Type {jtype} ({JammerTypeGAN(jtype).name}): "
                  f"avg power = {10*np.log10(avg_power):.1f} dB")
        
        # Save
        gan.save("/home/claude/nx-mimosa-unified/python/ew/export/adversarial_gan.pt")
        print("\n✅ GAN saved!")
    else:
        print("\nPyTorch not available. Using simple adversarial generator...")
        gen = SimpleAdversarialGenerator(signal_length=512)
        
        for jtype in range(4):
            samples = gen.generate_adversarial(jtype, n_samples=10)
            avg_power = np.mean(np.abs(samples) ** 2)
            print(f"  Type {jtype}: avg power = {10*np.log10(avg_power):.1f} dB")
    
    # Test SNR degradation
    print("\nSNR Degradation Analysis:")
    simple_gen = SimpleAdversarialGenerator(512)
    target = np.exp(2j * np.pi * 0.1 * np.arange(512))  # Example target
    
    for jtype in range(4):
        jammer = simple_gen.generate_adversarial(jtype, 1)[0]
        degradation = simple_gen.compute_snr_degradation(target, jammer)
        print(f"  Type {jtype}: {degradation:.1f} dB degradation")
    
    print("\n✅ Adversarial jammer generation complete!")


if __name__ == "__main__":
    main()
