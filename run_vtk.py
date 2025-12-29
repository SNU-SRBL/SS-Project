#!/usr/bin/env python3

import os
import time
import argparse
import numpy as np
from typing import Tuple, List
from sensor.sensor_srbl import FingerSensor

# VTK imports
import vtkmodules.vtkInteractionStyle
import vtkmodules.vtkRenderingOpenGL2
from vtkmodules.vtkCommonColor import vtkNamedColors
from vtkmodules.vtkIOGeometry import vtkSTLReader
from vtkmodules.vtkFiltersSources import vtkArrowSource
from vtkmodules.vtkRenderingCore import (
    vtkActor,
    vtkPolyDataMapper,
    vtkProperty,
    vtkRenderWindow,
    vtkRenderWindowInteractor,
    vtkRenderer
)


def get_filename() -> str:
    """Parse command line arguments to get STL filename."""
    description = 'Read an STL file.'
    parser = argparse.ArgumentParser(description=description,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('filename', nargs='?', default='sensor.STL', help='Path to STL file (e.g., RN.stl)')
    args = parser.parse_args()
    filename = os.path.join('sensor', args.filename)
    return filename

def create_stl_actor(colors: vtkNamedColors) -> vtkActor:
    """Create an actor for the sensor STL model."""
    filename = get_filename()


    reader = vtkSTLReader()
    reader.SetFileName(filename)

    mapper = vtkPolyDataMapper()
    mapper.SetInputConnection(reader.GetOutputPort())

    actor_stl = vtkActor()
    actor_stl.SetMapper(mapper)
    prop = actor_stl.GetProperty()
    prop.SetDiffuse(0.8)
    prop.SetDiffuseColor(colors.GetColor3d('DimGray'))
    prop.SetSpecular(0.3)
    prop.SetSpecularPower(60.0)
    prop.SetOpacity(0.7) # Opacity

    actor_stl.SetPosition(0, 0, 0)
    actor_stl.SetOrientation(90, 0, 0) # 0, 90, 0
    actor_stl.SetScale(1.0, 1.0, 1.0) # Adjust the scale

    return actor_stl

class AxisArrows():
    """Create XYZ arrows representing force vectors."""

    def __init__(self, colors: vtkNamedColors, is_right: bool):
        self.colors = colors
        self.is_right = is_right
        self.arrow_source = self._create_arrow_source()
        self.mapper = self._create_mapper()

    def _create_arrow_source(self) -> vtkArrowSource:
        arrow = vtkArrowSource()
        arrow.SetTipLength(.35)
        arrow.SetTipRadius(0.12) # 0.1
        arrow.SetTipResolution(30)
        arrow.SetShaftRadius(0.06) # 0.03
        arrow.SetShaftResolution(30)
        return arrow
    
    def _create_mapper(self) -> vtkPolyDataMapper:
        mapper = vtkPolyDataMapper()
        mapper.SetInputConnection(self.arrow_source.GetOutputPort())
        return mapper

    def _create_property(self, color_name: str) -> vtkProperty:
        prop = vtkProperty()
        prop.SetDiffuse(0.7)
        prop.SetSpecular(0.4)
        prop.SetSpecularPower(20)
        prop.SetColor(self.colors.GetColor3d(color_name))
        return prop

    def create_actors(self) -> Tuple[vtkActor, vtkActor, vtkActor]:
        """Return X, Y, Z axis actors with color and orientation."""
        actors = []
        for axis, color in zip(["x", "y", "z"], ["Blue", "Green", "Red"]):
            actor = vtkActor()
            actor.SetMapper(self.mapper)
            actor.SetProperty(self._create_property(color))
            actors.append(actor)

        actors[0].SetOrientation(0, 0, 0)     # 파랑 = X (그대로)
        actors[1].SetOrientation(0, 90, 0)    # 초록 = Y (X->Y)
        actors[2].SetOrientation(0, 0, 90)   # 빨강 = Z (X->Z)


        # actors[0].SetOrientation(0, 90, 0) # X
        # actors[1].SetOrientation(0, 0, 90) # Y

        position = (12, 2, -10) if self.is_right else (50, -60, 0)
        scale_z = -30 if self.is_right else 30

        for axis, actor in zip(["x", "y", "z"], actors):
            actor.SetPosition(*position)
            scale = [30, 30, 30]
            if axis == "z":
                scale[0] = scale_z
            actor.SetScale(*scale)

        return tuple(actors)


class ForceCallback():
    """Timer callback for visualizing real-time force data."""

    def __init__(self, 
                 steps: int, 
                 actors: List[vtkActor], 
                 iren: vtkRenderWindowInteractor, 
                 fsensor: FingerSensor):
        self.steps = steps
        self.actors = actors
        self.iren = iren
        self.fsensor = fsensor
        self.gain = 10
        self.timer_count = 0
        self.timerId = None

    def execute(self, obj, event) -> None:
        for _ in range(self.steps):
            sensor_data1, sensor_data2 = self.fsensor.read_force()
            print(f"{sensor_data1} {sensor_data2}")
            
            """
            right1 = x(forward) y(down) z(out)
            left2 = x(behind) y(down) z(out)
            """
            # Update scales based on sensor forces
            scale_values = np.array([*sensor_data1]) * self.gain
            scale_matrix = [
                scale_values[0] * -1, scale_values[1] * -1, scale_values[2],  # Right Sensor
            ] # Adjust here !!

            for actor, scale in zip(self.actors, scale_matrix):
                actor.SetScale(scale, scale, scale)

            self.iren.GetRenderWindow().Render()
            self.timer_count += 1

        if self.timerId:
            self.iren.DestroyTimer(self.timerId)


def main() -> None:
    """Main function to launch the force visualizer."""
    # Sensor
    fsensor = FingerSensor(port='COM3', force=True)

    # VTK
    colors = vtkNamedColors()

    # stl actor | Import Sensor STL file
    stl_actor= create_stl_actor(colors)

    # Arrow actors
    right_arrows = AxisArrows(colors, is_right=True).create_actors()
    # left_arrows = AxisArrows(colors, is_right=False).create_actors()
    all_actors = list(right_arrows) # + left_arrows)

    # Setup a renderer, render window, and interactor
    renderer = vtkRenderer()
    renderer.SetBackground(colors.GetColor3d("White"))

    renderWindow = vtkRenderWindow()
    renderWindow.SetWindowName("3-Axis Force Sensor Visualization")
    renderWindow.AddRenderer(renderer)
    renderWindow.SetSize(600, 600)

    renderWindowInteractor = vtkRenderWindowInteractor()
    renderWindowInteractor.SetRenderWindow(renderWindow)

    # Add the actor to the scene
    for actor in [stl_actor, *all_actors]:
        renderer.AddActor(actor)

    # Render and interact
    renderWindow.Render()
    renderer.GetActiveCamera().Zoom(0.8) # Camera View
    renderer.GetActiveCamera().Azimuth(-15) # Camera Orientation
    renderer.GetActiveCamera().Elevation(10) # Camera Orientation

    renderWindow.Render()

    # Initialize must be called prior to creating timer events.
    renderWindowInteractor.Initialize()

    # Sign up to receive TimerEvent
    cb = ForceCallback(steps=2_000_000, actors=all_actors, iren=renderWindowInteractor, fsensor=fsensor)
    renderWindowInteractor.AddObserver('TimerEvent', cb.execute)
    cb.timerId = renderWindowInteractor.CreateRepeatingTimer(500)

    # start the interaction and timer
    # renderWindow.Render() # Option
    renderWindowInteractor.Start()


if __name__ == '__main__':
    main()