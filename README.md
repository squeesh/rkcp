# Kerbal Space Program Thing
An application Derek wrote to do cool landings

### Code Organization

---
controller.py
>controls the ship in KSP
---

---
exec.py
>program entry point; loads the controller and runs it
---

---
manager.py
>Several manager classes for interpreting state & executing proper  landing
---

---
rendezvous_with_moon.py
>no clue; landing implementation logic is contained here
---

---
state.py
>collection of singleton objects used for managing game state for calculation purposes
---

---
util.py
>various utility and helper functions particularly useful when performing trajectory calculations
---

## Dependencies
* [numpy](http://www.numpy.org/)
* [sympy](https://www.sympy.org/)
* [KSP](https://www.kerbalspaceprogram.com/en/)

## Installation
* Running the code requires a python interpreter and KSP installation.
* load or create a virtual environment
* pip install -r requirements.txt
* While game is running click load script and select exec.py


## TODOs
1. Populate requirements.txt file
2. Isolate utility & math functions to their own file
3. Flesh out readme with clear design goals
4. Provide context or docstrings for those unfamiliar with Kerbal Space Program & its modding

## Code Monkey
* [squeesh](https://github.com/squeesh)
