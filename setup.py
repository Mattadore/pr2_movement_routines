from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['pr2_movement_routines'],
    package_dir={'': 'src'}
    #requires=['actionlib', 'geometry_msgs', 'moveit_msgs', 'pr2_controllers_msgs', 'rospy', 'tf', 'trajectory_msgs'],
    #scripts=['scripts/joint_state_listener_server.py']
)

setup(**d)

