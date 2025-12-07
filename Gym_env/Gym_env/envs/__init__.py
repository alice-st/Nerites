from gymnasium.envs.registration import register

# Register your custom environment
register(
    id="Nerites_mission_planning-v1",  # Unique ID for your environment
    entry_point="envs.base_env:make_env",
    max_episode_steps=10000,  # Maximum steps per episode
)