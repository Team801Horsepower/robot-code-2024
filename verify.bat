poetry run mypy src/
poetry run black src/
poetry run pylint src/ --disable=missing-docstring,invalid-name,missing-module-docstring