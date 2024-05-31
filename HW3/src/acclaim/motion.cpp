#include "acclaim/motion.h"
#include <fstream>
#include <iostream>
#include <utility>
#include <vector>
#define _USE_MATH_DEFINES
#include <math.h>

#include "simulation/kinematics.h"
#include "util/helper.h"

namespace acclaim {
Motion::Motion() {}
Motion::Motion(const util::fs::path &amc_file, std::unique_ptr<Skeleton> &&_skeleton) noexcept
    : skeleton(std::move(_skeleton)) {
    postures.reserve(1024);
    if (!this->readAMCFile(amc_file)) {
        std::cerr << "Error in reading AMC file, this object is not initialized!" << std::endl;
        std::cerr << "You can call readAMCFile() to initialize again" << std::endl;
        postures.resize(0);
    }
}

const std::unique_ptr<Skeleton> &Motion::getSkeleton() const { return skeleton; }

Motion::Motion(const Motion &other) noexcept
    : skeleton(std::make_unique<Skeleton>(*other.skeleton)), postures(other.postures) {}

Motion::Motion(Motion &&other) noexcept : skeleton(std::move(other.skeleton)), postures(std::move(other.postures)) {}

Motion::Motion(const Motion &other, int startIdx, int endIdx) : skeleton(std::make_unique<Skeleton>(*other.skeleton)) {
    if (startIdx < 0 || startIdx >= other.postures.size() || endIdx <= startIdx || endIdx > other.postures.size()) {
        throw std::out_of_range("Invalid start or end index");
    }

    postures.assign(other.postures.begin() + startIdx, other.postures.begin() + endIdx);
}

Motion &Motion::operator=(const Motion &other) noexcept {
    if (this != &other) {
        skeleton.reset();
        skeleton = std::make_unique<Skeleton>(*other.skeleton);
        postures = other.postures;
    }
    return *this;
}

Motion &Motion::operator=(Motion &&other) noexcept {
    if (this != &other) {
        skeleton = std::move(other.skeleton);
        postures = std::move(other.postures);
    }
    return *this;
}

void Motion::remove(int begin, int end) {
    assert(end >= begin);
    postures.erase(postures.begin() + begin, postures.begin() + end);
}

void Motion::concatenate(Motion &m2) {
    for (int i = 0; i < m2.getFrameNum(); i++)
        postures.insert(postures.end(), m2.postures[i]);
}

Posture& Motion::getPosture(int FrameNum) { 
    return postures[FrameNum]; 
}

std::vector<Posture> Motion::getPostures() { 
    return postures; 
}

void Motion::setPosture(int FrameNum, const Posture &InPosture) {
    postures[FrameNum] = InPosture; 
}

int Motion::getFrameNum() const { return static_cast<int>(postures.size()); }

void Motion::forwardkinematics(int frame_idx) {
    kinematics::forwardSolver(postures[frame_idx], skeleton->getBonePointer(0));
    skeleton->setModelMatrices();
}

void Motion::transform(Eigen::Vector4d &newFacing, const Eigen::Vector4d &newPosition) {
    // **TODO**
    // Task: Transform the whole motion segment so that the root bone of the first posture(first frame) 
    //       of the motion is located at newPosition, and its facing be newFacing.
    //       The whole motion segment must remain continuous.

    //1. Get the initial position and orientation of the root bone at frame 0
    Eigen::Vector4d p = postures[0].bone_translations[0];
    Eigen::Vector4d r = postures[0].bone_rotations[0];

    // 2. Compute the position difference D
    Eigen::Vector4d d = newPosition - p;

    // 3. Convert the initial and new facing directions to rotation matrices
    Eigen::Quaternion initialRotation = util::rotateDegreeZYX(r);
    Eigen::Quaternion newRotation = util::rotateDegreeZYX(newFacing);
    Eigen::Matrix3d initialRotationMatrix = initialRotation.toRotationMatrix();
    Eigen::Matrix3d newRotationMatrix = newRotation.toRotationMatrix();

     // 4. Extract the unit vectors for the initial and new facing directions
    Eigen::Vector3d initialDirection = initialRotationMatrix.row(2);
    Eigen::Vector3d newDirection = newRotationMatrix.row(2);

    // 5. Compute the angle theta_y for the y-axis rotation
    double theta_y = atan2(newDirection.z(), newDirection.x()) - atan2(initialDirection.z(), initialDirection.x());

    // 6. Compute the rotation matrix R for the y-axis rotation manually
    Eigen::Matrix3d R = Eigen::AngleAxisd(theta_y, Eigen::Vector3d::UnitY()).toRotationMatrix();

    // 7. Traverse through each posture and update its root bone position and rotation
    for (Posture &posture : postures) {
        Eigen::Vector4d current_pos = posture.bone_translations[0];
        Eigen::Vector4d current_rot = posture.bone_rotations[0];
        
        Eigen::Quaternion current_rotation_quaternion = util::rotateDegreeZYX(current_rot);
        Eigen::Matrix3d current_rotation_matrix = current_rotation_quaternion.toRotationMatrix();
        Eigen::Matrix3d updated_rotation_matrix = R * current_rotation_matrix;
        Eigen::Vector3d updated_euler_angles = updated_rotation_matrix.eulerAngles(2, 1, 0).reverse();
        Eigen::Vector3d updated_rotation_degrees = updated_euler_angles * (180.0 / M_PI);
        posture.bone_rotations[0].head<3>() = updated_rotation_degrees;

        Eigen::Vector3d p_diff = (current_pos.head<3>() - p.head<3>());
        Eigen::Vector3d p_rotate = R * p_diff;
        Eigen::Vector3d Position = p_rotate + p.head<3>() + d.head<3>();
        posture.bone_translations[0].head<3>() = Position;
    }
}

Motion blend(Motion bm1, Motion bm2, const std::vector<double> &weight) {
    // **TODO**
    // Task: Return a motion segment that blends bm1 and bm2.
    //       bm1: tail of m1, bm2: head of m2
    //       You can assume that m2's root position and orientation is aleady aligned with m1 before blending.
    //       In other words, m2.transform(...) will be called before m1.blending(m2, blendWeight, blendWindowSize) is
    //       called

    /// bm1, bm2 = 50
    int blendWindowsSize = bm1.getFrameNum();
    //int blendWindowsSize = 30;
    Motion blendMotion = bm1;
    //std::cout << "---------------WindowSize--------------\n";
    //std::cout << blendWindowsSize << "\n";
    //std::cout << "---------------WindowSize--------------\n";
    //for (int i = 0; i < weight.size(); i++) {
    //    std::cout << weight[i] << " ";
    //}
    //std::cout << "\n";


    for (int i = 0; i < blendWindowsSize; i++) {
        acclaim::Posture pos1 = bm1.getPosture(i);
        acclaim::Posture pos2 = bm2.getPosture(i);
        double w = weight[i];
        double ease_in_ease_out;
        //double ease_in_ease_out = (1 / 2) * sin((i / (blendWindowsSize - 1)) * M_PI - (M_PI / 2));
        Posture blendPosture(pos1.bone_rotations.size());
        for (size_t j = 0; j < pos1.bone_translations.size(); j++) {
            if (w < 0.5) {
                ease_in_ease_out = 2 * w * w;
            } else {
                ease_in_ease_out = -2 * w * w + 4 * w - 1;
            }
            blendPosture.bone_translations[j] = pos1.bone_translations[j] * (1 - ease_in_ease_out) + pos2.bone_translations[j] * ease_in_ease_out;

            // Interpolate bone rotations using SLERP
            for (size_t j = 0; j < pos1.bone_rotations.size(); j++) {
                Eigen::Quaternion r1 = util::rotateDegreeZYX(pos1.bone_rotations[j]);
                Eigen::Quaternion r2 = util::rotateDegreeZYX(pos2.bone_rotations[j]);
                Eigen::Quaternion r_interpolated = r1.slerp(ease_in_ease_out, r2);
                // Convert quaternion to Euler angles in XYZ order
                Eigen::Vector3d eulerAnglesXYZ = r_interpolated.toRotationMatrix().eulerAngles(2, 1,0).reverse();
                Eigen::Vector3d eulerAnglesXYZDeg = eulerAnglesXYZ * (180.0 / M_PI);
                blendPosture.bone_rotations[j].head<3>() = eulerAnglesXYZDeg;
            }
        }
        blendMotion.setPosture(i, blendPosture);
    }

    return blendMotion;
}

Motion Motion::blending(Motion &m2, const std::vector<double> &blendWeight, int blendWindowSize) {
    // do blending
    Motion bm1(*this, this->getFrameNum() - blendWindowSize, this->getFrameNum());
    Motion bm2(m2, 0, blendWindowSize);
    Motion bm = blend(bm1, bm2, blendWeight);
    return bm;
}


bool Motion::readAMCFile(const util::fs::path &file_name) {
    // Open AMC file
    std::ifstream input_stream(file_name);
    // Check if file successfully opened
    if (!input_stream) {
        std::cerr << "Failed to open " << file_name << std::endl;
        return false;
    }
    // There are (NUM_BONES_IN_ASF_FILE - 2) moving bones and 2 dummy bones (lhipjoint and rhipjoint)
    int movable_bones = skeleton->getMovableBoneNum();
    // Ignore header
    input_stream.ignore(1024, '\n');
    input_stream.ignore(1024, '\n');
    input_stream.ignore(1024, '\n');
    int frame_num;
    std::string bone_name;
    while (input_stream >> frame_num) {
        auto &&current_posture = postures.emplace_back(skeleton->getBoneNum());
        for (int i = 0; i < movable_bones; ++i) {
            input_stream >> bone_name;
            const Bone &bone = *skeleton->getBonePointer(bone_name);
            int bone_idx = bone.idx;
            Eigen::Vector4d bone_rotation = Eigen::Vector4d::Zero();
            Eigen::Vector4d bone_translation = Eigen::Vector4d::Zero();
            if (bone.doftx) {
                input_stream >> bone_translation[0];
            }
            if (bone.dofty) {
                input_stream >> bone_translation[1];
            }
            if (bone.doftz) {
                input_stream >> bone_translation[2];
            }
            if (bone.dofrx) {
                input_stream >> bone_rotation[0];
            }
            if (bone.dofry) {
                input_stream >> bone_rotation[1];
            }
            if (bone.dofrz) {
                input_stream >> bone_rotation[2];
            }
            current_posture.bone_rotations[bone_idx] = std::move(bone_rotation);
            current_posture.bone_translations[bone_idx] = std::move(bone_translation);
            if (bone_idx == 0) {
                current_posture.bone_translations[bone_idx] *= skeleton->getScale();
            }
        }
    }
    input_stream.close();
    std::cout << frame_num << " samples in " << file_name.string() << " are read" << std::endl;
    return true;
}
void Motion::render(graphics::Program *program) const { skeleton->render(program); }

}  // namespace acclaim
