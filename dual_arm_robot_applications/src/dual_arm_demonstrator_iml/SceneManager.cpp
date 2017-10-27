//
// Created by Daniel Höltgen on 20.10.16.
//

#include "dual_arm_demonstrator_iml/SceneManager.h"

using namespace dual_arm_demonstrator_iml;

SceneManager::SceneManager(ros::NodeHandle &nh) : nh_(nh) {
    // Planning Scene Publisher
    planning_scene_diff_publisher_ = nh_.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);
    while(planning_scene_diff_publisher_.getNumSubscribers() < 1) {
        ros::WallDuration sleep_t(0.5);
        sleep_t.sleep();
    }

    // constants
    box_.type = box_.BOX;
    box_.dimensions.resize(3);
    box_.dimensions[0] = 0.21;
    box_.dimensions[1] = 0.105;
    box_.dimensions[2] = 0.105;
}

void SceneManager::addBox(std::string id, geometry_msgs::Pose pose, std::string link_name, std::string frame_id) {
    // TODO: attach object?
    // Define the attached object message
    moveit_msgs::AttachedCollisionObject attached_object;

    attached_object.link_name = link_name;
    attached_object.object.header.frame_id = frame_id;
    attached_object.object.id = id;

    attached_object.object.primitives.push_back(box_);
    attached_object.object.primitive_poses.push_back(pose);

    // Define operation: add
    attached_object.object.operation = attached_object.object.ADD;

    ROS_INFO("Adding %s into the world", id.c_str());
    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(attached_object.object);
    planning_scene.is_diff = true;
    planning_scene_diff_publisher_.publish(planning_scene);
    sleep(1);
}

void SceneManager::addShelf(std::string id, geometry_msgs::Pose pose, std::string link_name, std::string frame_id) {
    // generate object from mesh
    moveit_msgs::CollisionObject co;
    shapes::Mesh* m = shapes::createMeshFromResource("package://dual_arm_robot_applications/meshes/shelf/shelf.stl");
    shape_msgs::Mesh co_mesh;
    shapes::ShapeMsg co_mesh_msg;
    shapes::constructMsgFromShape(m,co_mesh_msg);
    co_mesh = boost::get<shape_msgs::Mesh>(co_mesh_msg);

    // setup pose
    co.id = id;
    co.header.frame_id = frame_id;
    co.meshes.resize(1);
    co.meshes[0] = co_mesh;
    co.mesh_poses.resize(1);
    co.mesh_poses[0] = pose;

    // prepare to publish
    //co.meshes.push_back(co_mesh);
    //co.mesh_poses.push_back(co.mesh_poses[0]);
    co.operation = co.ADD;
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(co);

    // add object into world
    ROS_INFO("Adding %s into the world", id.c_str());
    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(co);
    planning_scene.is_diff = true;
    planning_scene_diff_publisher_.publish(planning_scene);
    sleep(1);
}

void SceneManager::setupScene() {
    // add shelf
    geometry_msgs::Pose shelf_pose;
    shelf_pose.position.x = 0.400;
    shelf_pose.position.y = -0.850;
    shelf_pose.position.z = 0.009;
    KDL::Rotation shelf_rot;
    shelf_rot.DoRotZ(-3.14/4);
    shelf_rot.GetQuaternion(shelf_pose.orientation.x, shelf_pose.orientation.y, shelf_pose.orientation.z, shelf_pose.orientation.w);
    addShelf("shelf", shelf_pose, "table_ground", "table_ground");

    // add box1
    geometry_msgs::Pose box1_pose;
    box1_pose.position.x = 0.25 - box_.dimensions[1]/2 - box_.dimensions[1];
    box1_pose.position.y = 0.25 - box_.dimensions[0]/2;
    box1_pose.position.z = 0.0155+box_.dimensions[2]/2;
    KDL::Rotation box1_rot;
    box1_rot.DoRotZ(3.14/2);
    box1_rot.GetQuaternion(box1_pose.orientation.x, box1_pose.orientation.y, box1_pose.orientation.z, box1_pose.orientation.w);
    dual_arm_demonstrator_iml::SceneManager::addBox("box1", box1_pose, "shelf", "shelf");

    // add box2
    geometry_msgs::Pose box2_pose;
    box2_pose.position.x = 0.25 - box_.dimensions[1]/2;
    box2_pose.position.y = 0.25 - box_.dimensions[0]/2;
    box2_pose.position.z = 0.0155+box_.dimensions[2]/2;
    KDL::Rotation box2_rot;
    box2_rot.DoRotZ(3.14/2);
    box2_rot.GetQuaternion(box2_pose.orientation.x, box2_pose.orientation.y, box2_pose.orientation.z, box2_pose.orientation.w);
    dual_arm_demonstrator_iml::SceneManager::addBox("box2", box2_pose, "shelf", "shelf");

    // add box3
    geometry_msgs::Pose box3_pose;
    box3_pose.position.x = 0.25 - box_.dimensions[1];
    box3_pose.position.y = 0.25 - 3*box_.dimensions[1]/2;
    box3_pose.position.z = 0.0155+box_.dimensions[2]/2+box_.dimensions[2]+0.001;
    KDL::Rotation box3_rot;
    box3_rot.GetQuaternion(box3_pose.orientation.x, box3_pose.orientation.y, box3_pose.orientation.z, box3_pose.orientation.w);
    dual_arm_demonstrator_iml::SceneManager::addBox("box3", box3_pose, "shelf", "shelf");

    // add box4
    geometry_msgs::Pose box4_pose;
    box4_pose.position.x = 0.04 + box_.dimensions[1]/2;
    box4_pose.position.y = 0.04 + box_.dimensions[2]/2;
    box4_pose.position.z = 0.0155+0.52/2+0.0155/2+box_.dimensions[0]/2;
    KDL::Rotation box4_rot;
    box4_rot.DoRotY(-3.14/2);
    box4_rot.GetQuaternion(box4_pose.orientation.x, box4_pose.orientation.y, box4_pose.orientation.z, box4_pose.orientation.w);
    dual_arm_demonstrator_iml::SceneManager::addBox("box4", box4_pose, "shelf", "shelf");

    // add box5
    geometry_msgs::Pose box5_pose;
    box5_pose.position.x = 0.04 + box_.dimensions[1]/2 + box_.dimensions[1];
    box5_pose.position.y = 0.04 + box_.dimensions[0]/2;
    box5_pose.position.z = 0.0155+0.52/2+0.0155/2+box_.dimensions[2]/2;
    KDL::Rotation box5_rot;
    box5_rot.DoRotZ(3.14/2);
    box5_rot.GetQuaternion(box5_pose.orientation.x, box5_pose.orientation.y, box5_pose.orientation.z, box5_pose.orientation.w);
    dual_arm_demonstrator_iml::SceneManager::addBox("box5", box5_pose, "shelf", "shelf");

    // add box6
    geometry_msgs::Pose box6_pose;
    box6_pose.position.x = 0.02 + box_.dimensions[0]/2;
    box6_pose.position.y = 0.02 + box_.dimensions[1]/2 + box_.dimensions[1];
    box6_pose.position.z = 0.0155+0.52+0.0155+box_.dimensions[2]/2;
    KDL::Rotation box6_rot;
    box6_rot.GetQuaternion(box6_pose.orientation.x, box6_pose.orientation.y, box6_pose.orientation.z, box6_pose.orientation.w);
    dual_arm_demonstrator_iml::SceneManager::addBox("box6", box6_pose, "shelf", "shelf");

    // add box7
    geometry_msgs::Pose box7_pose;
    box7_pose.position.x = 0.02 + box_.dimensions[0]/2;
    box7_pose.position.y = 0.02 + box_.dimensions[1]/2 + box_.dimensions[1];
    box7_pose.position.z = 0.0155+0.52+0.0155 + box_.dimensions[2]/2 + box_.dimensions[2] + 0.001;
    KDL::Rotation box7_rot;
    box7_rot.GetQuaternion(box7_pose.orientation.x, box7_pose.orientation.y, box7_pose.orientation.z, box7_pose.orientation.w);
    dual_arm_demonstrator_iml::SceneManager::addBox("box7", box7_pose, "shelf", "shelf");
}

void SceneManager::setupSceneLift() {
    // add box1
    geometry_msgs::Pose box1_pose;
    box1_pose.position.x = 0.55 + box_.dimensions[0]/2;
    box1_pose.position.y = -0.95 + box_.dimensions[1]/2 + box_.dimensions[1];
    box1_pose.position.z = 0.0001+box_.dimensions[2]/2;
    KDL::Rotation box1_rot;
    box1_rot.DoRotZ(-3.14/4);
    box1_rot.GetQuaternion(box1_pose.orientation.x, box1_pose.orientation.y, box1_pose.orientation.z, box1_pose.orientation.w);
    dual_arm_demonstrator_iml::SceneManager::addBox("box1", box1_pose, "table_ground", "table_ground");

    // add box2
    /*
    geometry_msgs::Pose box2_pose;
    box2_pose.position.x = 0.55 + box_.dimensions[0]/2;
    box2_pose.position.y = -0.95 + box_.dimensions[1]/2 + box_.dimensions[1];
    box2_pose.position.z = 0.0001+box_.dimensions[2]/2 + box_.dimensions[2];
    KDL::Rotation box2_rot;
    box2_rot.DoRotZ(-3.14/4);
    box2_rot.GetQuaternion(box2_pose.orientation.x, box2_pose.orientation.y, box2_pose.orientation.z, box2_pose.orientation.w);
    dual_arm_demonstrator_iml::SceneManager::addBox("box2", box2_pose, "table_ground", "table_ground");*/
}

void SceneManager::setupSceneLiftCO() { //with collision object
    // add box1
    geometry_msgs::Pose box1_pose;
    box1_pose.position.x = 0.55 + box_.dimensions[0]/2;
    box1_pose.position.y = -0.95 + box_.dimensions[1]/2 + box_.dimensions[1];
    box1_pose.position.z = 0.0001+box_.dimensions[2]/2;
    KDL::Rotation box1_rot;
    //box1_rot.DoRotZ(-3.14/4);
    box1_rot.GetQuaternion(box1_pose.orientation.x, box1_pose.orientation.y, box1_pose.orientation.z, box1_pose.orientation.w);
    dual_arm_demonstrator_iml::SceneManager::addBox("box1", box1_pose, "table_ground", "table_ground");

    // add box2
    geometry_msgs::Pose box2_pose;
    box2_pose.position.x = 0.55 + box_.dimensions[0]/2;
    box2_pose.position.y = -1.0 + box_.dimensions[1]/2 + box_.dimensions[1];
    box2_pose.position.z = 0.0001+ 0.30 + 0.07 + box_.dimensions[2];
    KDL::Rotation box2_rot;
    box2_rot.DoRotZ(-3.14/2);
    box2_rot.GetQuaternion(box2_pose.orientation.x, box2_pose.orientation.y, box2_pose.orientation.z, box2_pose.orientation.w);
    dual_arm_demonstrator_iml::SceneManager::addBox("box2", box2_pose, "table_ground", "table_ground");

    // add
    /*
    geometry_msgs::Pose box3_pose;
    box3_pose.position.x = 0.55 + box_.dimensions[0]/2;
    box3_pose.position.y = -1.1 + box_.dimensions[1]/2 + box_.dimensions[1];
    box3_pose.position.z = 0.0001+ 0.6 + box_.dimensions[2];
    KDL::Rotation box3_rot;
    box3_rot.DoRotZ(-3.14/2);
    box3_rot.GetQuaternion(box3_pose.orientation.x, box3_pose.orientation.y, box3_pose.orientation.z, box3_pose.orientation.w);
    dual_arm_demonstrator_iml::SceneManager::addBox("box3", box3_pose, "table_ground", "table_ground");*/

    // add box4
    geometry_msgs::Pose box4_pose;
    box4_pose.position.x = 0.55 + box_.dimensions[0]/2;
    box4_pose.position.y = -0.8 + box_.dimensions[1]/2 + box_.dimensions[1];
    box4_pose.position.z = 0.0001+ 0.9 + box_.dimensions[2];
    KDL::Rotation box4_rot;
    box4_rot.DoRotZ(-3.14/2);
    box4_rot.GetQuaternion(box4_pose.orientation.x, box4_pose.orientation.y, box4_pose.orientation.z, box4_pose.orientation.w);
    dual_arm_demonstrator_iml::SceneManager::addBox("box4", box4_pose, "table_ground", "table_ground");

    // add box5
    geometry_msgs::Pose box5_pose;
    box5_pose.position.x = 0.55 + box_.dimensions[0]/2;
    box5_pose.position.y = -1.2 + box_.dimensions[1]/2 + box_.dimensions[1];
    box5_pose.position.z = 0.0001+ 0.6 + 0.15 + box_.dimensions[2];
    KDL::Rotation box5_rot;
    box5_rot.DoRotZ(-3.14/2);
    box5_rot.GetQuaternion(box5_pose.orientation.x, box5_pose.orientation.y, box5_pose.orientation.z, box5_pose.orientation.w);
    dual_arm_demonstrator_iml::SceneManager::addBox("box5", box5_pose, "table_ground", "table_ground");
}