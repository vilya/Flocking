VH_Flocking - a flocking simulator for Nuke particles.
======================================================

This is a Particle node for Nuke 6.3 which simulates flocking behaviour.

It applies a few simple rules to each particle and it's from the combination of
these that the flocking behaviour emerges:
1. Each particles will try to move towards the centre of the set of particles.
2. All particles will try to remain at least a small distance away from all
   other particles.
3. Particles will try to match the velocity of all the other particles.

These are the basic rules of a flocking system. We also have the following
extra rules, to make it a bit more controllable:
4. The particles will steer towards a common location.
5. Particles will try to avoid certain other locations.
6. There's an upper limit to the speed that particles can travel at.


Compiling
=========

Linux
-----

- Edit the Makefile:
  - change the NUKE_VERSION_MAJOR and NUKE_VERSION_MINOR variables to match the
    version of Nuke that you're compiling against.
  - change the path on line 12 to match the root of your Nuke install.

- Run make:

    OSTYPE=linux-gnu make dirs
    OSTYPE=linux-gnu make

  The 'make dirs' command is only needed the first time you build the plugin;
  it creates the 'build' and 'dist' directories that will contain the
  intermediate files and the final plugin respectively.


Mac
---

- Edit the Makefile:
  - change the NUKE_VERSION_MAJOR and NUKE_VERSION_MINOR variables to match the
    version of Nuke that you're compiling against.
  - change the path on line 16 to match the root of your Nuke install.

- Run make:

    make dirs
    make

  The 'make dirs' command is only needed the first time you build the plugin;
  it creates the 'build' and 'dist' directories that will contain the
  intermediate files and the final plugin respectively.


Windows
-------

No idea, sorry. I've only tried building on Mac and Linux. Nuke ships with a
sample project for Visual Studio which may be enough to get you started.


Contact
=======

If you have any questions, feedback or requests, feel free to email me:

  Vilya Harvey
  <vilya.harvey@gmail.com>

