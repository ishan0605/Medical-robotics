# URDF importer for Slicer
#
# This script imports a universal robot description file (URDF) into 3D Slicer scene.
# This model can then be animated by updating the joint transforms by interactively modifying the transforms in 3D views
# or remotely via OpenIGTLink.
#
# Usage: Copy-paste this script into the Python console in 3D Slicer. The script automatically downloads an example URDF from github.
# You can use your own URDF file by setting `rootPath` and `urdfFilePath` to the path of your URDF file.

# Get URDF file.
import SampleData
downloadedFolder = SampleData.downloadFromURL(
    fileNames="RobotDescription.zip",
    uris="https://github.com/justagist/franka_panda_description/archive/refs/heads/master.zip")[0]
rootPath = downloadedFolder + "/franka_panda_description-master" # root path for models
urdfFilePath = rootPath + "/robots/panda_arm.urdf"  # full path of the .urdf file to import

# Parse robot description file   
import xml.etree.ElementTree as ET
# Parse XML data from a file
tree = ET.parse(urdfFilePath)
robot = tree.getroot()
if robot.tag != "robot":
    raise ValueError("Invalid URDF file")

# Add all links and joints to the scene
nodes = {}
for link in robot:
    name = link.get("name")
    if link.tag == "link":
        try:
            stlFilePath = rootPath + "/" + link.find('collision').find('geometry').find('mesh').attrib["filename"]
            # Use RAS coordinate system to avoid model conversion from LPS to RAS (we can transform the entire robot as a whole later if needed)
            modelNode = slicer.modules.models.logic().AddModel(stlFilePath, slicer.vtkMRMLStorageNode.CoordinateSystemRAS)
        except:
            # No mesh found, add a sphere
            sphere = vtk.vtkSphereSource()
            sphere.SetRadius(0.01)
            modelNode = slicer.modules.models.logic().AddModel(sphere.GetOutputPort())
        modelNode.SetName(name)
        nodes[name] = { "type": "link", "model": modelNode}
    
    elif link.tag == "joint":
        jointTransformNode = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLTransformNode", name)
        nodes[name] = { "type": "joint", "transform": jointTransformNode}
        if link.get("type") == "fixed":
            # do not create a display node, the transform does not have to be editable
            pass
        else:
            # make the transform interactively editable in 3D views
            jointTransformNode.CreateDefaultDisplayNodes()
            displayNode = jointTransformNode.GetDisplayNode()
            displayNode.SetEditorVisibility(True)
            displayNode.SetEditorSliceIntersectionVisibility(False)
            displayNode.SetEditorTranslationEnabled(False)
            if link.get("type") == "revolute":
                # <axis xyz="0 0 1"/>
                rotationAxis = [float(x) for x in link.find("axis").get("xyz").split()]
                if rotationAxis == [1, 0, 0]:
                    displayNode.SetRotationHandleComponentVisibility3D(True, False, False, False)
                elif rotationAxis == [0, 1, 0]:
                    displayNode.SetRotationHandleComponentVisibility3D(False, True, False, False)
                elif rotationAxis == [0, 0, 1]:
                    displayNode.SetRotationHandleComponentVisibility3D(False, False, True, False)
                else:
                    raise ValueError(f"Unsupported rotation axis {rotationAxis}")
            else:
                # TODO: implement translation and other joint types
                raise ValueError(f"Unsupported joint type {link.get('type')}")

# Create hierarchy
for joint in robot.findall("joint"):
    name = joint.get("name")

    parentName = joint.find("parent").get("link")
    if parentName:
        parent = nodes[parentName]
        if parent["type"] != "link":
            raise ValueError(f"Parent of joint {name} is not a link")
        jointToParentTransformNode = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLTransformNode", f"{name} to {parentName}")
        nodes[jointToParentTransformNode.GetName()] = { "type": "transform", "transform": jointToParentTransformNode}
        jointToParentTransformNode.SetAndObserveTransformNodeID(parent["model"].GetTransformNodeID())
        # <origin rpy="-1.57079632679 0 0" xyz="0 0 0"/>
        transformToParent = vtk.vtkTransform()
        rpy = [vtk.vtkMath.DegreesFromRadians(float(x)) for x in joint.find("origin").get("rpy").split()]
        xyz = [float(x) for x in joint.find("origin").get("xyz").split()]
        transformToParent.Translate(xyz)
        transformToParent.RotateY(rpy[1])
        transformToParent.RotateX(rpy[0])
        transformToParent.RotateZ(rpy[2])
        jointToParentTransformNode.SetMatrixTransformToParent(transformToParent.GetMatrix())
        nodes[name]["transform"].SetAndObserveTransformNodeID(jointToParentTransformNode.GetID())
    
    # iterate through all children
    for child in joint.findall("child"):
        childName = child.get("link")
        child = nodes[childName]
        if child["type"] != "link":
            raise ValueError(f"Child of joint {name} is not a link")
        childModelNode = child["model"]
        childModelNode.SetAndObserveTransformNodeID(nodes[name]["transform"].GetID())

# Add all top-level nodes under root transform.
# This is needed to transform from URDF coordinate world coordinate system, which uses meters, to Slicer coordinate system, which uses millimeters.
# It is also useful to be able to position the robot anywhere within the scene
robotToWorldTransformNode = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLTransformNode", "Robot")
robotToWorldTransform = vtk.vtkTransform()
robotToWorldTransform.Scale(1000, 1000, 1000)  # convert from meters (URDF) to millimeters (Slicer)
robotToWorldTransformNode.SetMatrixTransformToParent(robotToWorldTransform.GetMatrix())
for nodeName in nodes:
    if nodes[nodeName]["type"] == "link":
        node = nodes[nodeName]["model"]
    elif nodes[nodeName]["type"] == "joint" or nodes[nodeName]["type"] == "transform":
        node = nodes[nodeName]["transform"]
    if not node.GetParentTransformNode():
        node.SetAndObserveTransformNodeID(robotToWorldTransformNode.GetID())
