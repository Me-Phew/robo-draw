import logging

from pydantic_settings import BaseSettings, SettingsConfigDict


class RoboDrawerSettings(BaseSettings):
    def __init__(self, **data):
        super().__init__(**data)

    model_config = SettingsConfigDict(validate_default=True, env_nested_delimiter="__")

    # DEBUG OPTIONS
    LOG_LEVEL: int = logging.INFO

    # GENERAL SETTINGS
    ENCODING: str = "utf-8"
