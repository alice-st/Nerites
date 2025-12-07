# utils/normalize_env.py
import gymnasium as gym
import numpy as np

class RunningMeanStd:
    def __init__(self, shape=(), eps=1e-8):
        self.mean  = np.zeros(shape, dtype=np.float64)
        self.var   = np.ones(shape,  dtype=np.float64)
        self.count = float(eps)

    def update(self, x):
        x = np.asarray(x, dtype=np.float64)

        # Treat a single sample vector as a batch of size 1
        if x.shape == self.mean.shape:
            x = x[None, ...]  # -> (1, *shape)

        batch_mean  = x.mean(axis=0)
        batch_var   = x.var(axis=0)
        batch_count = x.shape[0]

        delta      = batch_mean - self.mean
        tot_count  = self.count + batch_count
        new_mean   = self.mean + delta * (batch_count / tot_count)

        m_a = self.var * self.count
        m_b = batch_var * batch_count
        M2  = m_a + m_b + (delta**2) * self.count * batch_count / tot_count
        new_var = M2 / tot_count

        self.mean, self.var, self.count = new_mean, new_var, tot_count


class NormalizeEnv(gym.Wrapper):
    """
    Z-score normalize observations and (optionally) scale rewards.

    reward_norm:
      - "return": divide by running std of discounted returns (VecNormalize-style)
      - "reward": divide by running std of per-step rewards (more stable for spikes)
      - "none":   no reward normalization (obs still normalized)
    """
    def __init__(
        self, env,
        obs_clip=5.0, rew_clip=5.0, gamma=0.99, eps=1e-8, train=True,
        reward_norm="reward", init_ret_std=None, init_rew_std=1
    ):
        super().__init__(env)
        self.obs_rms = RunningMeanStd(shape=self.observation_space.shape)

        # We keep both; only the chosen mode is updated/used.
        self.ret_rms = RunningMeanStd(shape=())
        self.rew_rms = RunningMeanStd(shape=())

        # Optional seeding of std to avoid crazy early scales
        if init_ret_std is not None:
            self.ret_rms.var   = float(init_ret_std)**2
            self.ret_rms.count = 1e6
        if init_rew_std is not None:
            self.rew_rms.var   = float(init_rew_std)**2
            self.rew_rms.count = 1e6

        self.obs_clip = float(obs_clip)
        self.rew_clip = float(rew_clip)
        self.gamma = float(gamma)
        self.eps = float(eps)
        self.train = bool(train)
        self.reward_norm = str(reward_norm).lower()
        assert self.reward_norm in {"return", "reward", "none"}

        self._ret = 0.0  # running discounted return (for "return" mode)

    # ---- observation ----
    def _norm_obs(self, obs):
        if self.train:
            self.obs_rms.update(obs)
        std = np.sqrt(self.obs_rms.var + self.eps)
        o = (obs - self.obs_rms.mean) / std
        return np.clip(o, -self.obs_clip, self.obs_clip)

    # ---- reward ----
    def _norm_rew(self, rew):
        if self.reward_norm == "none":
            return float(rew)

        if self.reward_norm == "return":
            if self.train:
                self.ret_rms.update(np.array([self._ret], dtype=np.float64))
            std = float(np.sqrt(self.ret_rms.var + self.eps))

        else:  # "reward"
            if self.train:
                self.rew_rms.update(np.array([rew], dtype=np.float64))
            std = float(np.sqrt(self.rew_rms.var + self.eps))

        r = rew / max(std, self.eps)
        return float(np.clip(r, -self.rew_clip, self.rew_clip))

    # ---- gym api ----
    def reset(self, **kwargs):
        obs, info = self.env.reset(**kwargs)
        self._ret = 0.0
        return self._norm_obs(obs), info

    def step(self, action):
        obs, rew, terminated, truncated, info = self.env.step(action)
        # Update running return *before* normalization (for "return" mode)
        self._ret = self._ret * self.gamma + float(rew)
        return self._norm_obs(obs), self._norm_rew(float(rew)), terminated, truncated, info

    # ---- persist stats ----
    def save_stats(self, path):
        np.savez(
            path,
            obs_mean=self.obs_rms.mean, obs_var=self.obs_rms.var, obs_count=self.obs_rms.count,
            ret_var=self.ret_rms.var,   ret_count=self.ret_rms.count,
            rew_var=self.rew_rms.var,   rew_count=self.rew_rms.count,
            reward_norm=np.array(self.reward_norm)
        )

    def load_stats(self, path):
        d = np.load(path, allow_pickle=True)
        self.obs_rms.mean  = d["obs_mean"]
        self.obs_rms.var   = d["obs_var"]
        self.obs_rms.count = float(d["obs_count"])
        self.ret_rms.var   = d["ret_var"]
