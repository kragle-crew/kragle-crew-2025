# Kragle Crew 2025

## How to load these files onto a Spike Prime core

## Prerequisites

- A recent version of [`python`](https://www.python.org/). This readme is tested with python 3
- A lego spike prime core flashed to [pybricks](https://pybricks.com/) firmware
  - [Instructions](https://pybricks.com/learn/getting-started/install-pybricks/)
- [Visual Studio Code](https://code.visualstudio.com/). Note that this is free.
- The [`pybricksdev`](https://github.com/pybricks/pybricksdev) utility

### Manually

These instructions were tested on macos. Windows equivalents may be found in links.

Make a [venv](https://docs.python.org/3/library/venv.html). In this example I have named mine `.venv`, and am using python 3.

```shell
python3 -m venv .venv
```

Activate it.

```shell
source .venv/bin/activate
```

Install the dependancies.

```shell
pip install -r requirements.txt
```

Load the python code via [pybricksdev](https://github.com/pybricks/pybricksdev).

```shell
pybricksdev run ble example.py
```

### Via VSCode

//todo

## Load multiple programs

example: <https://github.com/pybricks/support/issues/861>
