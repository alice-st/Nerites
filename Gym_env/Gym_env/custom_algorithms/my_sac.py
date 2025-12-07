import gymnasium as gym
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.optim import Adam
from torch.distributions import Normal
import collections
import random
import pandas as pd


# =========================
# Replay Buffer
# =========================
class ReplayBuffer:
    def __init__(self, buffer_limit: int):
        self.buffer = collections.deque(maxlen=buffer_limit)

    def put(self, transition):
        self.buffer.append(transition)

    def sample(self, n: int):
        mini_batch = random.sample(self.buffer, n)
        s_lst, a_lst, r_lst, s_prime_lst, term_mask_lst, trunc_mask_lst = [], [], [], [], [], []

        for transition in mini_batch:
            s, a, r, s_prime, term_mask, trunc_mask = transition
            s_lst.append(np.array(s, dtype=np.float32).flatten())
            a_lst.append(np.array(a, dtype=np.float32).flatten())
            r_lst.append([r])
            s_prime_lst.append(np.array(s_prime, dtype=np.float32).flatten())
            term_mask_lst.append([term_mask])
            trunc_mask_lst.append([trunc_mask])

        s_array = np.stack(s_lst)
        a_array = np.stack(a_lst)
        r_array = np.array(r_lst, dtype=np.float32)
        s_prime_array = np.stack(s_prime_lst)
        term_mask_array = np.array(term_mask_lst, dtype=np.float32)
        trunc_mask_array = np.array(trunc_mask_lst, dtype=np.float32)

        return (
            torch.tensor(s_array, dtype=torch.float),
            torch.tensor(a_array, dtype=torch.float),
            torch.tensor(r_array, dtype=torch.float),
            torch.tensor(s_prime_array, dtype=torch.float),
            torch.tensor(term_mask_array, dtype=torch.float),
            torch.tensor(trunc_mask_array, dtype=torch.float),
        )

    def size(self) -> int:
        return len(self.buffer)


# =========================
# Actor
# =========================
class Actor(nn.Module):
    """
    Gaussian policy with tanh squashing, then scaled to env.action_space.low/high.
    """

    def __init__(self, state_dim: int, action_dim: int, action_low, action_high):
        super().__init__()
        self.fc1 = nn.Linear(state_dim, 256)
        self.fc2 = nn.Linear(256, 256)
        self.mu = nn.Linear(256, action_dim)
        self.log_std = nn.Linear(256, action_dim)

        # store bounds as buffers so they move with the model to GPU if needed
        self.register_buffer(
            "action_low",
            torch.as_tensor(action_low, dtype=torch.float32).view(1, -1),
        )
        self.register_buffer(
            "action_high",
            torch.as_tensor(action_high, dtype=torch.float32).view(1, -1),
        )

        self.LOG_STD_MIN = -20
        self.LOG_STD_MAX = 2

    def forward(self, state: torch.Tensor):
        x = F.relu(self.fc1(state))
        x = F.relu(self.fc2(x))
        mu = self.mu(x)
        log_std = self.log_std(x)
        log_std = torch.clamp(log_std, self.LOG_STD_MIN, self.LOG_STD_MAX)
        std = torch.exp(log_std)
        return mu, std

    def _squash_and_scale(self, z: torch.Tensor):
        """
        z: pre-tanh sample (or mean), shape [B, action_dim]
        Returns:
            action in [low, high], and u = tanh(z) in [-1, 1]
        """
        u = torch.tanh(z)  # in [-1, 1]
        # map from [-1, 1] -> [low, high]
        action = self.action_low + (u + 1.0) * 0.5 * (self.action_high - self.action_low)
        return action, u

    def sample(self, state: torch.Tensor):
        """
        Stochastic action + log_prob (for SAC).
        """
        mu, std = self.forward(state)
        normal = Normal(mu, std)
        z = normal.rsample()              # reparameterized sample
        action, u = self._squash_and_scale(z)

        # SAC log_prob correction for tanh
        log_prob = normal.log_prob(z) - torch.log(1 - u.pow(2) + 1e-6)
        log_prob = log_prob.sum(dim=1, keepdim=True)
        return action, log_prob

    def mean_action(self, state: torch.Tensor):
        """
        Deterministic action for evaluation: use mean, then tanh+scale.
        """
        mu, _ = self.forward(state)
        action, _ = self._squash_and_scale(mu)
        return action


# =========================
# Critic (double Q)
# =========================
class Critic(nn.Module):
    def __init__(self, state_dim: int, action_dim: int):
        super().__init__()
        self.fc1 = nn.Linear(state_dim + action_dim, 256)
        self.fc2 = nn.Linear(256, 256)
        self.fc3 = nn.Linear(256, 1)

        self.fc4 = nn.Linear(state_dim + action_dim, 256)
        self.fc5 = nn.Linear(256, 256)
        self.fc6 = nn.Linear(256, 1)

    def forward(self, state: torch.Tensor, action: torch.Tensor):
        sa = torch.cat([state, action], 1)

        q1 = F.relu(self.fc1(sa))
        q1 = F.relu(self.fc2(q1))
        q1 = self.fc3(q1)

        q2 = F.relu(self.fc4(sa))
        q2 = F.relu(self.fc5(q2))
        q2 = self.fc6(q2)

        return q1, q2

    def Q1(self, state: torch.Tensor, action: torch.Tensor):
        sa = torch.cat([state, action], 1)
        q1 = F.relu(self.fc1(sa))
        q1 = F.relu(self.fc2(q1))
        q1 = self.fc3(q1)
        return q1


# =========================
# SAC Agent
# =========================
class SAC:
    def __init__(
        self,
        env,
        buffer_size: int = 1_000_000,
        batch_size: int = 256,
        gamma: float = 0.99,
        tau: float = 0.005,
        alpha: float = 0.2,
        lr: float = 3e-4,
    ):
        self.env = env
        self.state_dim = env.observation_space.shape[0]
        self.action_dim = env.action_space.shape[0]

        # full low/high vectors from the Box, not just a scalar
        self.action_low = env.action_space.low
        self.action_high = env.action_space.high

        self.replay_buffer = ReplayBuffer(buffer_size)
        self.batch_size = batch_size
        self.gamma = gamma
        self.tau = tau
        self.alpha = alpha

        self.actor = Actor(
            self.state_dim, self.action_dim, self.action_low, self.action_high
        )
        self.critic = Critic(self.state_dim, self.action_dim)
        self.critic_target = Critic(self.state_dim, self.action_dim)
        self.critic_target.load_state_dict(self.critic.state_dict())

        self.actor_optimizer = Adam(self.actor.parameters(), lr=lr)
        self.critic_optimizer = Adam(self.critic.parameters(), lr=lr)

        # for optional logging
        self.episode_log = pd.DataFrame(
            columns=["episode", "steps", "reward", "avg_reward"]
        )

    # --------------------
    # Action selection
    # --------------------
    def select_action(self, state, evaluate: bool = False):
        if isinstance(state, tuple):
            state = state[0]

        state = np.array(state, dtype=np.float32).flatten()
        if state.shape[0] != self.state_dim:
            raise ValueError(
                f"Expected state dimension {self.state_dim}, got {state.shape[0]}"
            )

        state_t = torch.as_tensor(state, dtype=torch.float32).unsqueeze(0)

        if evaluate:
            # deterministic policy for evaluation
            action = self.actor.mean_action(state_t)
        else:
            # stochastic policy for training
            action, _ = self.actor.sample(state_t)

        action_np = action.detach().cpu().numpy()[0]
        return action_np

    # --------------------
    # SAC update step
    # --------------------
    def update(self):
        if self.replay_buffer.size() < self.batch_size:
            return

        state, action, reward, next_state, term_mask, trunc_mask = self.replay_buffer.sample(
            self.batch_size
        )

        with torch.no_grad():
            next_action, next_log_prob = self.actor.sample(next_state)
            q1_next_target, q2_next_target = self.critic_target(next_state, next_action)
            min_q_next_target = (
                torch.min(q1_next_target, q2_next_target) - self.alpha * next_log_prob
            )

            # Combine terminated and truncated for target Q calculation
            target_Q = reward + (
                (1 - term_mask) * (1 - trunc_mask) * self.gamma * min_q_next_target
            )

        q1, q2 = self.critic(state, action)
        critic_loss = F.mse_loss(q1, target_Q) + F.mse_loss(q2, target_Q)

        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        self.critic_optimizer.step()

        pi, log_pi = self.actor.sample(state)
        q1_pi, q2_pi = self.critic(state, pi)
        min_Q = torch.min(q1_pi, q2_pi)
        actor_loss = (self.alpha * log_pi - min_Q).mean()

        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        self.actor_optimizer.step()

        # soft update of target network
        for param, target_param in zip(
            self.critic.parameters(), self.critic_target.parameters()
        ):
            target_param.data.copy_(
                self.tau * param.data + (1 - self.tau) * target_param.data
            )

    # --------------------
    # Helpers for eval env (NormalizeEnv)
    # --------------------
    def _sync_norm_stats(self, eval_env):
        """
        Copy normalization stats from train env to eval env, if they exist.

        If eval_env is the same object as self.env, this is effectively a no-op,
        but it's still safe.
        """
        if not hasattr(self.env, "obs_rms") or not hasattr(eval_env, "obs_rms"):
            return

        eval_env.obs_rms.mean = self.env.obs_rms.mean.copy()
        eval_env.obs_rms.var = self.env.obs_rms.var.copy()
        eval_env.obs_rms.count = self.env.obs_rms.count

        if hasattr(self.env, "ret_rms") and hasattr(eval_env, "ret_rms"):
            eval_env.ret_rms.mean = self.env.ret_rms.mean.copy()
            eval_env.ret_rms.var = self.env.ret_rms.var.copy()
            eval_env.ret_rms.count = self.env.ret_rms.count

    def _evaluate(self, eval_env=None, max_steps: int = 50, eval_episodes: int = 5):
        """
        Run deterministic evaluation and return (mean, std) reward.

        If eval_env is None, fall back to self.env (same robot / same Gazebo).
        """
        if eval_env is None:
            eval_env = self.env

        self._sync_norm_stats(eval_env)
        rewards = []

        for _ in range(eval_episodes):
            out = eval_env.reset()
            state = out[0] if isinstance(out, tuple) else out
            ep_reward = 0.0

            for step in range(max_steps):
                action = self.select_action(state, evaluate=True)
                next_out = eval_env.step(action)
                next_state, reward, terminated, truncated = next_out[:4]
                ep_reward += reward
                state = next_state
                if terminated or truncated:
                    break

            rewards.append(ep_reward)

        rewards = np.array(rewards, dtype=np.float32)
        return float(rewards.mean()), float(rewards.std())

    # --------------------
    # Training loop
    # --------------------
    def train(
        self,
        num_episodes: int = 1000,
        max_steps: int = 50,
        save_path: str = "sac_model.pth",
        best_model_path: str = None,
        eval_env=None,
        eval_freq: int = 10,
        eval_episodes: int = 5,
        rewards_log_path: str = None,
    ):
        episode_rewards = []
        best_mean_reward = -np.inf

        if rewards_log_path is not None:
            self.episode_log = pd.DataFrame(
                columns=["episode", "steps", "reward", "avg_reward"]
            )

        for episode in range(num_episodes):
            out = self.env.reset()
            state = out[0] if isinstance(out, tuple) else out

            episode_reward = 0.0
            ep_len = 0

            for step in range(max_steps):
                action = self.select_action(state)
                next_out = self.env.step(action)
                next_state, reward, terminated, truncated = next_out[:4]

                if isinstance(next_state, tuple):
                    next_state = next_state[0]

                term_mask = 1.0 if terminated else 0.0
                trunc_mask = 1.0 if truncated else 0.0

                # store transition
                self.replay_buffer.put(
                    (state, action, reward, next_state, term_mask, trunc_mask)
                )

                # update SAC after every env step
                self.update()

                state = next_state
                episode_reward += reward
                ep_len += 1

                if terminated or truncated:
                    break

            episode_rewards.append(episode_reward)
            avg_reward = float(np.mean(episode_rewards))
            print(f"Episode {episode}, Reward: {episode_reward:.2f}, Avg: {avg_reward:.2f}")

            # log to DataFrame / CSV if requested
            if rewards_log_path is not None:
                self.episode_log.loc[len(self.episode_log)] = {
                    "episode": episode,
                    "steps": ep_len,
                    "reward": episode_reward,
                    "avg_reward": avg_reward,
                }
                self.episode_log.to_csv(rewards_log_path, index=False)

            # periodic checkpoint of current model
            if (episode + 1) % 50 == 0:
                torch.save(
                    {
                        "actor": self.actor.state_dict(),
                        "critic": self.critic.state_dict(),
                        "critic_target": self.critic_target.state_dict(),
                    },
                    save_path,
                )

            # periodic evaluation
            if (episode + 1) % eval_freq == 0:
                mean_r, std_r = self._evaluate(
                    eval_env=eval_env,  # can be None => uses self.env
                    max_steps=max_steps,
                    eval_episodes=eval_episodes,
                )
                print(f"[Eval] Episode {episode}: Mean reward {mean_r:.2f} +/- {std_r:.2f}")

                if best_model_path is not None and mean_r > best_mean_reward:
                    best_mean_reward = mean_r
                    print(f"[Eval] New best mean reward {best_mean_reward:.2f}, saving to {best_model_path}")
                    torch.save(
                        {
                            "actor": self.actor.state_dict(),
                            "critic": self.critic.state_dict(),
                            "critic_target": self.critic_target.state_dict(),
                        },
                        best_model_path,
                    )

        # final save of last model
        torch.save(
            {
                "actor": self.actor.state_dict(),
                "critic": self.critic.state_dict(),
                "critic_target": self.critic_target.state_dict(),
            },
            save_path,
        )

    # --------------------
    # Testing / inference
    # --------------------
    def test(self, num_episodes: int = 10, max_steps: int = 50):
        """
        Test the current policy on self.env (same robot / same Gazebo).
        """
        success = 0
        for episode in range(num_episodes):
            out = self.env.reset()
            state = out[0] if isinstance(out, tuple) else out
            episode_reward = 0.0

            for step in range(max_steps):
                action = self.select_action(state, evaluate=True)
                next_out = self.env.step(action)
                next_state, reward, terminated, truncated = next_out[:4]
                if isinstance(next_state, tuple):
                    next_state = next_state[0]

                episode_reward += reward
                state = next_state

                if terminated:
                    success += 1
                if terminated or truncated:
                    break

            print(f"[Test] Episode {episode}, Steps: {step}, Reward: {episode_reward:.2f}")

        print(f"Successful episodes: {success}")
