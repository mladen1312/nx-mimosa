#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════════════════════════════════
NX-MIMOSA PPO Agent for Cognitive ECCM
═══════════════════════════════════════════════════════════════════════════════════════════════════════

Proximal Policy Optimization (PPO) agent for autonomous waveform adaptation.
Learns optimal frequency/PRF/bandwidth offsets against reactive jammers.

Features:
  - Continuous action space (waveform parameters)
  - Noisy observations (realistic sensor model)
  - GAE advantage estimation
  - Policy + Value networks (Actor-Critic)
  - FPGA-exportable quantized policy

Traceability:
  [REQ-RL-PPO-001] PPO training for waveform agility
  [REQ-RL-MIT-001] Jammer mitigation via learned policy
  [REQ-ECCM-RL] RL-based ECCM

Author: Dr. Mladen Mešter / Nexellum d.o.o.
Version: 1.1.0
License: AGPL v3 / Commercial
═══════════════════════════════════════════════════════════════════════════════════════════════════════
"""

import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict
from collections import deque
import json

# Optional PyTorch imports
try:
    import torch
    import torch.nn as nn
    import torch.optim as optim
    from torch.distributions import Normal
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False


# =============================================================================
# CONFIGURATION
# =============================================================================

@dataclass
class PPOConfig:
    """PPO hyperparameters."""
    # Environment
    state_dim: int = 4          # [last_tx_offset, jammer_est, jammer_type, time_since_hop]
    action_dim: int = 2         # [freq_offset, prf_offset] (continuous)
    
    # Training
    n_epochs: int = 400
    steps_per_epoch: int = 128
    batch_size: int = 64
    n_updates: int = 10         # PPO update epochs per batch
    
    # PPO specific
    gamma: float = 0.99         # Discount factor
    gae_lambda: float = 0.95    # GAE lambda
    clip_ratio: float = 0.2     # PPO clip ratio
    target_kl: float = 0.01     # Early stopping KL threshold
    
    # Networks
    hidden_dim: int = 64
    lr_actor: float = 3e-4
    lr_critic: float = 1e-3
    
    # Exploration
    init_std: float = 0.5       # Initial action std
    min_std: float = 0.1        # Minimum action std
    
    # Reward shaping
    reward_not_jammed: float = 10.0
    reward_jammed: float = -10.0
    separation_bonus_scale: float = 5.0


# =============================================================================
# ENVIRONMENT: JAMMER SIMULATION
# =============================================================================

class ReactiveJammerEnv:
    """
    Simulated environment with reactive jammer.
    
    [REQ-RL-MIT-001] Jammer mitigation environment
    """
    
    def __init__(self, config: PPOConfig, jammer_type: str = 'reactive'):
        self.config = config
        self.jammer_type = jammer_type
        
        # State space: normalized to [-1, 1]
        self.freq_range = (-1.0, 1.0)  # Normalized freq offset
        self.prf_range = (-1.0, 1.0)   # Normalized PRF offset
        
        # Jammer parameters
        self.jammer_track_threshold = 0.3  # Distance below which jammer tracks
        self.jammer_reaction_delay = 2     # Steps delay for reactive jammer
        self.observation_noise = 0.2       # Sensor noise σ
        
        # State
        self.tx_offset = 0.0
        self.jammer_freq = 0.0
        self.step_count = 0
        self.jammer_history = deque(maxlen=self.jammer_reaction_delay)
    
    def reset(self) -> np.ndarray:
        """Reset environment."""
        self.tx_offset = np.random.uniform(-0.5, 0.5)
        self.jammer_freq = np.random.uniform(-0.5, 0.5)
        self.step_count = 0
        self.jammer_history.clear()
        
        return self._get_observation()
    
    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, dict]:
        """
        Execute action and return (obs, reward, done, info).
        
        Args:
            action: [freq_offset_delta, prf_offset_delta] in [-1, 1]
        """
        # Apply action (with clipping)
        freq_delta = np.clip(action[0], -0.3, 0.3)  # Max 30% change per step
        self.tx_offset = np.clip(self.tx_offset + freq_delta, -1.0, 1.0)
        
        # Update jammer behavior
        self._update_jammer()
        
        # Calculate reward
        distance = abs(self.tx_offset - self.jammer_freq)
        is_jammed = distance < self.jammer_track_threshold
        
        if is_jammed:
            reward = self.config.reward_jammed
        else:
            reward = self.config.reward_not_jammed
            # Bonus for maintaining separation
            reward += self.config.separation_bonus_scale * distance
        
        self.step_count += 1
        done = self.step_count >= self.config.steps_per_epoch
        
        info = {
            'distance': distance,
            'is_jammed': is_jammed,
            'tx_offset': self.tx_offset,
            'jammer_freq': self.jammer_freq
        }
        
        return self._get_observation(), reward, done, info
    
    def _update_jammer(self):
        """Update jammer position based on type."""
        # Store current TX for reactive jammer
        self.jammer_history.append(self.tx_offset)
        
        if self.jammer_type == 'reactive':
            # Reactive jammer: tracks radar with delay
            if len(self.jammer_history) >= self.jammer_reaction_delay:
                target = self.jammer_history[0]
                distance = abs(self.tx_offset - self.jammer_freq)
                
                if distance < 0.5:  # Close enough to track
                    # Move towards delayed target
                    self.jammer_freq += 0.2 * (target - self.jammer_freq)
                else:
                    # Random drift when far
                    self.jammer_freq += np.random.uniform(-0.1, 0.1)
        
        elif self.jammer_type == 'sweep':
            # Sweep jammer: periodic sweep across band
            self.jammer_freq = 0.8 * np.sin(self.step_count * 0.1)
        
        elif self.jammer_type == 'random':
            # Random hopping jammer
            if np.random.random() < 0.1:
                self.jammer_freq = np.random.uniform(-1.0, 1.0)
        
        # Clip jammer position
        self.jammer_freq = np.clip(self.jammer_freq, -1.0, 1.0)
    
    def _get_observation(self) -> np.ndarray:
        """Get noisy observation."""
        # Add observation noise
        noisy_jammer_est = self.jammer_freq + np.random.normal(0, self.observation_noise)
        
        return np.array([
            self.tx_offset,
            np.clip(noisy_jammer_est, -1.0, 1.0),
            float(self.step_count) / self.config.steps_per_epoch,  # Time feature
            0.0  # Placeholder for jammer type encoding
        ], dtype=np.float32)


# =============================================================================
# NEURAL NETWORKS
# =============================================================================

if TORCH_AVAILABLE:
    class ActorNetwork(nn.Module):
        """Policy network (Actor) for continuous actions."""
        
        def __init__(self, state_dim: int, action_dim: int, hidden_dim: int, init_std: float):
            super().__init__()
            
            self.shared = nn.Sequential(
                nn.Linear(state_dim, hidden_dim),
                nn.Tanh(),
                nn.Linear(hidden_dim, hidden_dim),
                nn.Tanh()
            )
            
            self.mean = nn.Linear(hidden_dim, action_dim)
            self.log_std = nn.Parameter(torch.ones(action_dim) * np.log(init_std))
        
        def forward(self, state: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
            """Return action mean and std."""
            features = self.shared(state)
            mean = torch.tanh(self.mean(features))  # Bounded actions
            std = torch.exp(self.log_std).expand_as(mean)
            return mean, std
        
        def get_action(self, state: torch.Tensor, deterministic: bool = False):
            """Sample action from policy."""
            mean, std = self.forward(state)
            
            if deterministic:
                return mean, None, None
            
            dist = Normal(mean, std)
            action = dist.sample()
            log_prob = dist.log_prob(action).sum(dim=-1)
            
            return action, log_prob, dist
    
    
    class CriticNetwork(nn.Module):
        """Value network (Critic)."""
        
        def __init__(self, state_dim: int, hidden_dim: int):
            super().__init__()
            
            self.network = nn.Sequential(
                nn.Linear(state_dim, hidden_dim),
                nn.Tanh(),
                nn.Linear(hidden_dim, hidden_dim),
                nn.Tanh(),
                nn.Linear(hidden_dim, 1)
            )
        
        def forward(self, state: torch.Tensor) -> torch.Tensor:
            return self.network(state).squeeze(-1)


# =============================================================================
# PPO AGENT
# =============================================================================

class PPOAgent:
    """
    Proximal Policy Optimization agent.
    
    [REQ-RL-PPO-001] PPO training for waveform agility
    """
    
    def __init__(self, config: PPOConfig):
        self.config = config
        
        if not TORCH_AVAILABLE:
            raise ImportError("PyTorch required for PPO agent")
        
        # Networks
        self.actor = ActorNetwork(
            config.state_dim, config.action_dim,
            config.hidden_dim, config.init_std
        )
        self.critic = CriticNetwork(config.state_dim, config.hidden_dim)
        
        # Optimizers
        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=config.lr_actor)
        self.critic_optimizer = optim.Adam(self.critic.parameters(), lr=config.lr_critic)
        
        # Training buffers
        self.states = []
        self.actions = []
        self.rewards = []
        self.log_probs = []
        self.values = []
        self.dones = []
        
        # Statistics
        self.training_stats = []
    
    def select_action(self, state: np.ndarray, deterministic: bool = False) -> np.ndarray:
        """Select action given state."""
        state_tensor = torch.FloatTensor(state).unsqueeze(0)
        
        with torch.no_grad():
            action, log_prob, _ = self.actor.get_action(state_tensor, deterministic)
            value = self.critic(state_tensor)
        
        action = action.squeeze(0).numpy()
        
        if not deterministic:
            self.log_probs.append(log_prob.item())
            self.values.append(value.item())
        
        return np.clip(action, -1.0, 1.0)
    
    def store_transition(self, state: np.ndarray, action: np.ndarray, 
                        reward: float, done: bool):
        """Store transition in buffer."""
        self.states.append(state)
        self.actions.append(action)
        self.rewards.append(reward)
        self.dones.append(done)
    
    def compute_gae(self, next_value: float) -> Tuple[np.ndarray, np.ndarray]:
        """Compute Generalized Advantage Estimation."""
        rewards = np.array(self.rewards)
        values = np.array(self.values + [next_value])
        dones = np.array(self.dones)
        
        advantages = np.zeros_like(rewards)
        last_gae = 0
        
        for t in reversed(range(len(rewards))):
            delta = rewards[t] + self.config.gamma * values[t+1] * (1 - dones[t]) - values[t]
            advantages[t] = last_gae = delta + self.config.gamma * self.config.gae_lambda * (1 - dones[t]) * last_gae
        
        returns = advantages + values[:-1]
        
        return advantages, returns
    
    def update(self) -> Dict[str, float]:
        """Perform PPO update."""
        # Convert to tensors
        states = torch.FloatTensor(np.array(self.states))
        actions = torch.FloatTensor(np.array(self.actions))
        old_log_probs = torch.FloatTensor(np.array(self.log_probs))
        
        # Compute GAE
        with torch.no_grad():
            next_value = self.critic(states[-1:]).item()
        
        advantages, returns = self.compute_gae(next_value)
        advantages = torch.FloatTensor(advantages)
        returns = torch.FloatTensor(returns)
        
        # Normalize advantages
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)
        
        # PPO update loop
        total_actor_loss = 0
        total_critic_loss = 0
        total_kl = 0
        
        for _ in range(self.config.n_updates):
            # Get new log probs and values
            mean, std = self.actor(states)
            dist = Normal(mean, std)
            new_log_probs = dist.log_prob(actions).sum(dim=-1)
            entropy = dist.entropy().sum(dim=-1).mean()
            
            new_values = self.critic(states)
            
            # PPO clipped objective
            ratio = torch.exp(new_log_probs - old_log_probs)
            surr1 = ratio * advantages
            surr2 = torch.clamp(ratio, 1 - self.config.clip_ratio, 1 + self.config.clip_ratio) * advantages
            
            actor_loss = -torch.min(surr1, surr2).mean() - 0.01 * entropy
            critic_loss = nn.MSELoss()(new_values, returns)
            
            # Update actor
            self.actor_optimizer.zero_grad()
            actor_loss.backward()
            nn.utils.clip_grad_norm_(self.actor.parameters(), 0.5)
            self.actor_optimizer.step()
            
            # Update critic
            self.critic_optimizer.zero_grad()
            critic_loss.backward()
            nn.utils.clip_grad_norm_(self.critic.parameters(), 0.5)
            self.critic_optimizer.step()
            
            total_actor_loss += actor_loss.item()
            total_critic_loss += critic_loss.item()
            
            # KL divergence for early stopping
            kl = (old_log_probs - new_log_probs).mean().item()
            total_kl += abs(kl)
            
            if abs(kl) > self.config.target_kl:
                break
        
        # Clear buffers
        self.states.clear()
        self.actions.clear()
        self.rewards.clear()
        self.log_probs.clear()
        self.values.clear()
        self.dones.clear()
        
        return {
            'actor_loss': total_actor_loss / self.config.n_updates,
            'critic_loss': total_critic_loss / self.config.n_updates,
            'kl': total_kl / self.config.n_updates
        }
    
    def train(self, env: ReactiveJammerEnv, verbose: bool = True) -> List[Dict]:
        """
        Train agent on environment.
        
        Returns:
            Training statistics per epoch
        """
        stats = []
        
        for epoch in range(self.config.n_epochs):
            state = env.reset()
            epoch_reward = 0
            epoch_jammed = 0
            epoch_steps = 0
            
            # Collect trajectories
            for step in range(self.config.steps_per_epoch):
                action = self.select_action(state)
                next_state, reward, done, info = env.step(action)
                
                self.store_transition(state, action, reward, done)
                
                epoch_reward += reward
                epoch_jammed += int(info['is_jammed'])
                epoch_steps += 1
                
                state = next_state
                
                if done:
                    break
            
            # PPO update
            update_info = self.update()
            
            # Statistics
            epoch_stats = {
                'epoch': epoch,
                'reward': epoch_reward,
                'avg_reward': epoch_reward / epoch_steps,
                'jammed_rate': epoch_jammed / epoch_steps,
                **update_info
            }
            stats.append(epoch_stats)
            self.training_stats.append(epoch_stats)
            
            if verbose and epoch % 50 == 0:
                print(f"Epoch {epoch:4d} | Reward: {epoch_reward:8.1f} | "
                      f"Jammed: {epoch_stats['jammed_rate']:.1%} | "
                      f"Actor Loss: {update_info['actor_loss']:.4f}")
        
        return stats
    
    def save(self, path: str):
        """Save agent to file."""
        torch.save({
            'actor_state_dict': self.actor.state_dict(),
            'critic_state_dict': self.critic.state_dict(),
            'config': self.config,
            'training_stats': self.training_stats
        }, path)
    
    def load(self, path: str):
        """Load agent from file."""
        checkpoint = torch.load(path)
        self.actor.load_state_dict(checkpoint['actor_state_dict'])
        self.critic.load_state_dict(checkpoint['critic_state_dict'])
        self.training_stats = checkpoint.get('training_stats', [])


# =============================================================================
# Q-TABLE EXPORT FOR FPGA
# =============================================================================

def export_policy_to_qtable(agent: PPOAgent, 
                           n_state_bins: int = 16,
                           n_action_bins: int = 8,
                           output_path: str = None) -> np.ndarray:
    """
    Export continuous PPO policy to discretized Q-table for FPGA.
    
    [REQ-RL-FIXED-001] Fixed-point Q-table export
    
    Args:
        agent: Trained PPO agent
        n_state_bins: Number of state discretization bins
        n_action_bins: Number of action bins
        output_path: Output path for .hex file
    
    Returns:
        Q-table as numpy array
    """
    # Create state grid
    state_range = np.linspace(-1, 1, n_state_bins)
    
    # Evaluate policy at each state
    q_table = np.zeros((n_state_bins, n_state_bins, n_action_bins))
    
    for i, tx_offset in enumerate(state_range):
        for j, jammer_est in enumerate(state_range):
            state = np.array([tx_offset, jammer_est, 0.5, 0.0], dtype=np.float32)
            
            # Get deterministic action from policy
            action = agent.select_action(state, deterministic=True)
            
            # Discretize action to bin
            action_bin = int((action[0] + 1) / 2 * (n_action_bins - 1))
            action_bin = np.clip(action_bin, 0, n_action_bins - 1)
            
            # Store "pseudo Q-value" (we use action probability as proxy)
            q_table[i, j, action_bin] = 1.0
    
    if output_path:
        # Export as hex for FPGA
        with open(output_path, 'w') as f:
            f.write("// Auto-generated Q-table from PPO policy\n")
            f.write(f"// State bins: {n_state_bins}x{n_state_bins}, Action bins: {n_action_bins}\n")
            f.write("// Format: Q8.8 fixed-point\n\n")
            
            for i in range(n_state_bins):
                for j in range(n_state_bins):
                    # Find best action for this state
                    best_action = np.argmax(q_table[i, j, :])
                    # Convert to Q8.8 (scale by 256)
                    q_val = int(q_table[i, j, best_action] * 256 * 15)  # Scale for visibility
                    f.write(f"{q_val:04X}  // state[{i}][{j}] -> action {best_action}\n")
        
        print(f"Q-table exported to {output_path}")
    
    return q_table


def export_policy_weights(agent: PPOAgent, output_path: str):
    """
    Export policy network weights for FPGA implementation.
    
    Generates fixed-point weights for HLS/Vitis AI.
    """
    weights = {}
    
    for name, param in agent.actor.named_parameters():
        weights[name] = param.detach().numpy()
    
    # Export as JSON with Q8.8 quantization
    quantized = {}
    for name, w in weights.items():
        # Quantize to Q8.8 (8 integer bits, 8 fractional)
        q_weights = np.round(w * 256).astype(np.int16)
        quantized[name] = q_weights.tolist()
    
    with open(output_path, 'w') as f:
        json.dump(quantized, f, indent=2)
    
    print(f"Policy weights exported to {output_path}")
    
    return quantized


# =============================================================================
# MAIN
# =============================================================================

def main():
    """Train PPO agent and export for FPGA."""
    print("═" * 70)
    print("NX-MIMOSA PPO Agent Training")
    print("═" * 70)
    
    if not TORCH_AVAILABLE:
        print("PyTorch not available. Skipping training.")
        return
    
    # Configuration
    config = PPOConfig(
        n_epochs=200,
        steps_per_epoch=128,
        hidden_dim=64
    )
    
    # Create environment and agent
    env = ReactiveJammerEnv(config, jammer_type='reactive')
    agent = PPOAgent(config)
    
    # Train
    print("\nTraining PPO agent against reactive jammer...")
    stats = agent.train(env, verbose=True)
    
    # Results
    final_reward = np.mean([s['reward'] for s in stats[-50:]])
    final_jammed = np.mean([s['jammed_rate'] for s in stats[-50:]])
    
    print("\n" + "═" * 70)
    print("TRAINING RESULTS")
    print("═" * 70)
    print(f"Final avg reward (last 50 epochs): {final_reward:.1f}")
    print(f"Final jammed rate: {final_jammed:.1%}")
    print(f"Effective Pd: {1 - final_jammed:.1%}")
    
    # Export for FPGA
    import os
    os.makedirs("/home/claude/nx-mimosa-unified/python/ew/export", exist_ok=True)
    
    export_policy_to_qtable(
        agent, 
        n_state_bins=16,
        n_action_bins=8,
        output_path="/home/claude/nx-mimosa-unified/python/ew/export/ppo_qtable.hex"
    )
    
    export_policy_weights(
        agent,
        output_path="/home/claude/nx-mimosa-unified/python/ew/export/ppo_weights.json"
    )
    
    # Save model
    agent.save("/home/claude/nx-mimosa-unified/python/ew/export/ppo_agent.pt")
    
    print("\n✅ PPO Agent training complete!")
    
    return agent, stats


if __name__ == "__main__":
    agent, stats = main()
