@echo off

:frc_help
echo Usage: frc.cmd [verify ^| deploy ^| console ^| shell ^| rlds]
goto :eof

if "%1" == "" (
    call :frc_help
) else if "%1" == "verify" (
    echo Running black auto-formatter...
    poetry run black src/
    echo Running mypy static type checker...
    poetry run mypy src/
    echo Running pylint linter...
    poetry run pylint src/ --disable=missing-docstring,invalid-name,missing-module-docstring
) else if "%1" == "deploy" (
    cd src/
    poetry run robotpy deploy --skip-tests --team 801
    cd ..
) else if "%1" == "console" (
    poetry run netconsole 10.8.1.2
) else if "%1" == "shell" (
    poetry shell
) else if "%1" == "rlds" (
    ssh -o StrictHostKeyChecking=no admin@10.8.1.2 '/home/lvuser/rlds/deploy'
) else (
    call :frc_help
)
