# Work w/ the FETCH Robot
- All header files located in include directory
- In src directory: **crop.cpp is the unorganized codebase**, obj_recognition.cpp acts as the new main file which calls the rest of the classes
## As of Thursday, 7/29/21:
### Needs fixing:
- MOVEIT error: [ERROR] [1627590839.325785991, 15906.323000000]: Transform from frame '' to frame 'base_link' is not known ('' should be a link name or an attached body's id).
### Next Steps:
- Fetch can't move to the clusters, goals are acting weird -- hopefully will be fixed/easier to fix with solving the above error 
- With transforming the pointcloud, crop box is relative to base_link as well and needs adjusting
- Broadcast goal targets, if that doesn't work then publish and subscriber
- Octomap

