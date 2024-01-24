@echo off

if "%1" == "" (
    echo Usage: frc.bat [verify | deploy | console]
) else if "%1" == "verify" (
    echo Running black auto-formatter...
    poetry run black src/
    echo.
    echo Running mypy static type checker...
    poetry run mypy src/
    echo.
    echo Running pylint linter...
    poetry run pylint src/ --disable=missing-docstring,invalid-name,missing-module-docstring
) else if "%1" == "deploy" (
    cd src/
    poetry run robotpy deploy --skip-tests --team 801
) else if "%1" == "console" (
    poetry run netconsole 10.8.1.2
) else if "%1" == "shell" (
    poetry shell
) else if "%1" == "rlds" (
    ssh -o StrictHostKeyChecking=no admin@10.8.1.2 'sh /home/lvuser/rlds/deploy'
) else (
    echo Usage: frc.bat [verify | deploy | console]
)