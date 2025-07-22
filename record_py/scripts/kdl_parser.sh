git clone https://github.com/ros/kdl_parser.git
cd kdl_parser/kdl_parser_py
python3 setup.py install

# def _toKdlJoint(jnt):

#     fixed = lambda j, F: kdl.Joint(j.name, joint_type=kdl.Joint.Fixed)
#     rotational = lambda j, F: kdl.Joint(j.name, F.p, F.M * kdl.Vector(*j.axis), kdl.Joint.RotAxis)
#     translational = lambda j, F: kdl.Joint(j.name, F.p, F.M * kdl.Vector(*j.axis), kdl.Joint.TransAxis)

#     type_map = {
#             'fixed': fixed,
#             'revolute': rotational,
#             'continuous': rotational,
#             'prismatic': translational,
#             'floating': fixed,
#             'planar': fixed,
#             'unknown': fixed,
#             }

#     return type_map[jnt.type](jnt, _toKdlPose(jnt.origin))