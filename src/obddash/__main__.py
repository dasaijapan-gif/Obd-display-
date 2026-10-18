"""``python -m obddash`` - same as the ``obddash`` console script."""

from obddash.cli import main

if __name__ == "__main__":
    raise SystemExit(main())
