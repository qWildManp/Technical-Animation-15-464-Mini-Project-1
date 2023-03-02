#ifndef BROWSEMODE_HPP
#define BROWSEMODE_HPP

#include "Mode.hpp"

#include <Character/Character.hpp>
#include <Character/Skin.hpp>
#include <Library/Library.hpp>
#include <Library/matrix.hpp>
#include <vector>
#include <deque>
#include <utility>
#include <string>
#include <Character/pose_utils.hpp>


using std::deque;
using std::vector;
using std::pair;
using std::string;

class BrowseMode : public Mode {
public:
	BrowseMode();
	virtual ~BrowseMode();

	virtual void update(float const elapsed_time);

	
	virtual void handle_event(SDL_Event const &event);

	virtual void draw();
	virtual void CCDApproach();
	virtual void JacobianApproach();
	virtual void JacobianApproach_another();

	Vector3f camera;
	Vector3f target;
	bool track;
	bool enable_ccd;
	bool enable_jacobian;
	Character::Pose current_pose;
	Character::Pose current_active_pose;
	Character::Pose new_pose;
	unsigned int current_motion;
	float time;
	float play_speed;

	Character::Skin skin;
	Character::WorldBones wb;
	Character::Angles new_angles;
	float horizontal_movement;
	float vertical_movement;
	float height;
	float dynamic_scaler;
	float dynamic_scaler_another[3];
	float max_interation;
	Vector3f ballPos;
	int tick;
	int index_c;
	float previous_error;
	float previous_error_another[3];
};

#endif //BROWSEMODE_HPP
