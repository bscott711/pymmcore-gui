set windows-powershell

# run the main application
run:
    uv run python -m pymmcore_gui

# create application bundle using pyinstaller
bundle:
    uv run pyinstaller app/mmgui.spec --clean --noconfirm --log-level INFO

# lint all files with pre-commit
lint:
    uv run pre-commit run --all-files

# Automatically fix all ruff linting and formatting issues
fix:
    uv run ruff check --fix .
    uv run ruff format .

# run tests
test:
    uv run pytest
