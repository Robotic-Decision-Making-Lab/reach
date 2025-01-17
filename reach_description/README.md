# Reach Description

This package contains the various description files for Reach Robotics devices.

## Configurations

The configuration files are stored in the `/config` directory and are
organized according to their respective device.

## URDFs

All xacro files are stored in the `/description` directory and are organized
according to their respective device.

> [!NOTE]
> For manipulator devices, the top-level configuration xacros (i.e.,
> `<device>.config.xacro`) are categorized according to the end-effector used.
> For instance, the xacro for the Reach Alpha 5 manipulator equipped with the
> standard jaws end effector is located at
> `/description/alpha_5/standard_jaws/alpha_5.config.xacro`.

## Mesh Files

All 3D mesh files are located in the `/meshes` directory and are organized
according to their respective device.

## RViz Configurations

All RViz configuration files are stored in the `/rviz` directory.
