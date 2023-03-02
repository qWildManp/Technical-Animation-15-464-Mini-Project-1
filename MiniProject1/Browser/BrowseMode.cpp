#include "BrowseMode.hpp"

#include <Character/Draw.hpp>
#include <Character/pose_utils.hpp>
#include <Library/Library.hpp>
#include <Character/skin_utils.hpp>
#include <Library/matrix.hpp>
#include <Graphics/Graphics.hpp>
#include <Graphics/Font.hpp>
#include <Vector/VectorGL.hpp>

#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <sys/stat.h>
#include <SDL.h>

using std::string;

using std::ostringstream;

using std::cout;
using std::cerr;
using std::endl;

using std::ofstream;

BrowseMode::BrowseMode() {
	camera = make_vector(10.0f, 10.0f, 10.0f);
	target = make_vector(0.0f, 0.0f, 0.0f);
	track = false;
	current_pose.clear();
	current_active_pose.clear();
	current_motion = 0;
	time = 0.0f;
	play_speed = 1.0f;
	horizontal_movement = 0;
	vertical_movement = 0;
	height = 0;
	ballPos = make_vector(0.0f, 0.0f, 0.0f);
	tick = 0;
	index_c = 21;
	dynamic_scaler = 1.0f;
	dynamic_scaler_another[0] = 1.0f;
	dynamic_scaler_another[1] = 1.0f;
	dynamic_scaler_another[2] = 1.0f;
	max_interation = 500;
	previous_error = -1.0f;
	previous_error_another[0] = -1.0f;
	previous_error_another[1] = -1.0f;
	previous_error_another[2] = -1.0f;
	enable_ccd = false;
	enable_jacobian = false;
}

BrowseMode::~BrowseMode() {
}

void BrowseMode::update(float const elapsed_time) {
	ballPos = make_vector(2 + horizontal_movement, 7 + height, -10 + vertical_movement);
	assert(current_motion < Library::motion_count());
	Library::Motion const &motion = Library::motion(current_motion);
	if (motion.frames() == 0) {
		cerr << "Motion from " << motion.filename << " has zero frames." << endl;
		return;
	}
	time = fmodf(elapsed_time * play_speed + time, motion.length());
	unsigned int frame = (unsigned int)(time / motion.skeleton->timestep);
	frame = 1;
	if (frame >= motion.frames()) frame = motion.frames() - 1;
	if (current_active_pose.root_position == make_vector(0.0f, 0.0f, 0.0f)) {
		motion.get_pose(frame, current_pose);
	}
	else {
		current_pose = current_active_pose;
	}
		
	Character::get_world_bones(current_pose, wb);
	motion.get_angles(frame, new_angles);
	//CCD || Jacobean Approach
	if (tick >= 10) {
		if(enable_ccd)
			CCDApproach();
		if(enable_jacobian)
			JacobianApproach();
		tick = 0;
	}
	tick++;
	
	//new current pos

	if (track) {
		Vector3f delta = current_pose.root_position - target;
		target += delta;
		camera += delta;
	}
}
void BrowseMode::CCDApproach() {
		
		Vector3f distance = wb.tips[21] - ballPos;
		if (length_squared(distance) < 0.001f) {// if cloase enough stop iterating
			return;
		}//2 vctors
		Vector3f joint_2_tip = wb.tips[21] - wb.bases[index_c];
		Vector3f joint_2_target = ballPos - wb.bases[index_c];
		cout << "tip" << wb.tips[21] << "target" << ballPos << "base" << wb.bases[21] << endl;
		joint_2_tip /= length(joint_2_tip);
		joint_2_target /= length(joint_2_target);
		Quatf quat = rotation(joint_2_tip, joint_2_target);
		Quatf curQ = current_pose.bone_orientations[index_c];
		Quatf result = multiply(quat, curQ);
		//cout << quat << "result quat";
		current_pose.bone_orientations[index_c] = result;
		current_active_pose = current_pose;
		index_c--;
		if (index_c <= 18) {
		index_c = 21;
		}
}
void BrowseMode::JacobianApproach_another() {// something wrong with it
	matrix delta_theta(9, 1);			
	Quatf rotation_quat[3];

	Vector3f distance = ballPos - wb.tips[21];
	matrix delta_e(3, 1);
	delta_e.setValue(distance.x, 0, 0);
	delta_e.setValue(distance.y, 1, 0);
	delta_e.setValue(distance.z, 2, 0);

	if (length_squared(distance) < 0.6) {
		return;
	}
	//set small push to rotate arm
	matrix Jacobian(3, 9);
		for (int axis = 0; axis < 3;axis++) {
			Quatf small_rot;
			small_rot.clear();
			if (axis == 0) {
				small_rot = multiply(rotation((float)(1 * M_PI / 180.0f), make_vector(1.0f, 0.0f, 0.0f)), small_rot);
			}
			else if (axis == 1) {
				small_rot = multiply(rotation((float)(1 * M_PI / 180.0f), make_vector(0.0f, 1.0f, 0.0f)), small_rot);
			}
			else if (axis == 2) {
				small_rot = multiply(rotation((float)(1 * M_PI / 180.0f), make_vector(0.0f, 0.0f, 1.0f)), small_rot);
			}
			small_rot = normalize(small_rot);
			cout << "quat_rot:" << small_rot << endl;

			
			Character::WorldBones tmp_wb;
			for (int j = 0; j < 3; j++) {
			Character::Pose tmp_pose = current_pose;
			Quatf currentRot = current_pose.bone_orientations[21- j];
			Quatf newRot = multiply(currentRot, small_rot);
			tmp_pose.bone_orientations[21- j] = newRot;
			Character::get_world_bones(tmp_pose, tmp_wb);
			//IK calculate error
			Vector3f error = tmp_wb.tips[21] - wb.tips[21];
			cout <<"tip_id:"<<21-j << "error:" << error << endl;
			if (previous_error_another[axis] != -1) {
				if (previous_error > length(error)) {
					if (dynamic_scaler_another[axis] > 0.01) {
						dynamic_scaler_another[axis] /= 4;
					}
				}
				else {
					if (dynamic_scaler_another[axis] < 20) {
						dynamic_scaler_another[axis] *= 4;
					}
				}
			}
			previous_error_another[axis] = length(error);
			Jacobian.setValue(error.x, 0, j * 3 + axis);
			Jacobian.setValue(error.y, 1, j * 3 + axis);
			Jacobian.setValue(error.z, 2, j * 3 + axis);
			}
			matrix JacCompose(9, 3);
			matrix JacDotJacComp(3, 3);
			matrix JacDotJacCompInv(3, 3);
			Jacobian.computeTranspose(&JacCompose);
			JacCompose.computeMatrixMul(&delta_e, &delta_theta);
			rotation_quat[axis].clear();
			for (int i = 0; i < 3;i++) {
				rotation_quat[axis] = multiply(rotation((float)(delta_theta.getValue(i * 3 + 0, 0) * dynamic_scaler_another[axis]*M_PI / 180.0f), make_vector(1.0f, 0.0f, 0.0f)), rotation_quat[axis]);
				rotation_quat[axis] = multiply(rotation((float)(delta_theta.getValue(i * 3 + 1, 0)  * dynamic_scaler_another[axis]*M_PI / 180.0f), make_vector(0.0f, 1.0f, 0.0f)), rotation_quat[axis]);
				rotation_quat[axis] = multiply(rotation((float)(delta_theta.getValue(i * 3 + 2, 0)  * dynamic_scaler_another[axis]*M_PI / 180.0f), make_vector(0.0f, 0.0f, 1.0f)), rotation_quat[axis]);
				rotation_quat[axis] = normalize(rotation_quat[axis]);
			}
			cout << "rotation quat:" << rotation_quat[axis] << endl;
	}
	//compute J
	for (int i = 0; i < 3;i++) {
		int index = 21 - i;
		Quatf curRot = current_pose.bone_orientations[index];
		Quatf result;
		result.clear();
		for (int j = 2;j > 0;j--) {
			result = multiply(result, rotation_quat[j]);
		}
		result = multiply(curRot, result);
		current_pose.bone_orientations[index] = result;
	}
	current_active_pose = current_pose;
}
void BrowseMode::JacobianApproach() { // Jacobian Approach
	matrix delta_theta(9, 1);
	Vector3f distance_vecter =  ballPos - wb.tips[21];
	matrix delta_e(3, 1);
	delta_e.setValue(distance_vecter.x, 0, 0);
	delta_e.setValue(distance_vecter.y, 1, 0);
	delta_e.setValue(distance_vecter.z, 2, 0);
	float distance = length(distance_vecter);
	if (distance*distance < 0.001f) {// if cloase enough stop iterating
		return;
	}
	matrix Jacobian(3, 9);
	for (int j = 0; j < 3; j++) {
		//set small push to rotate arm
		for (int axis = 0; axis < 3; axis++) {
		Quatf small_rot;
		small_rot.clear();
		if (axis == 0) {
			small_rot = multiply(rotation((float)(1.0f * M_PI / 180.0f), make_vector(1.0f, 0.0f, 0.0f)), small_rot);
		}
		else if (axis == 1) {
			small_rot = multiply(rotation((float)(1.0f * M_PI / 180.0f), make_vector(0.0f, 1.0f, 0.0f)), small_rot);
		}
		else if (axis == 2) {
			small_rot = multiply(rotation((float)(1.0f * M_PI / 180.0f), make_vector(0.0f, 0.0f, 1.0f)), small_rot);
		}
		small_rot = normalize(small_rot);
		cout << "quat_rot:" << small_rot << endl;

		Character::Pose tmp_pose = current_pose;
		Character::WorldBones tmp_wb;
		
		Quatf currentRot = current_pose.bone_orientations[21-j];
		Quatf newRot = multiply(currentRot, small_rot);
		tmp_pose.bone_orientations[21-j] = newRot;
		
		Character::get_world_bones(tmp_pose, tmp_wb);
		Vector3f delta_e_step =  tmp_wb.tips[21-j] - wb.tips[21-j];
		cout << "axis :" << j;
		cout << "joint :" << axis;
		Jacobian.setValue(delta_e_step.x, 0, j * 3 + axis);
		Jacobian.setValue(delta_e_step.y, 1, j * 3 + axis);
		Jacobian.setValue(delta_e_step.z, 2, j * 3 + axis);
		}
	}
	//compute J
	cout << "J :" << endl;
	Jacobian.printMatrix();


	//pesudoinvese J// partly now it is using Transpose J
	matrix JacTranspose(9, 3);
	matrix JacDotJacTrans(3, 3);
	matrix JacDotJacTransInv(3, 3);
	matrix JacPesudo(9,3);
	Jacobian.computeTranspose(&JacTranspose);// J transpose

	Jacobian.computeMatrixMul(&JacTranspose, &JacDotJacTrans); // J * J Transpose
	cout << "J * J Transpose :" << endl;
	JacDotJacTrans.printMatrix();

	JacDotJacTrans.invertMatrix(&JacDotJacTransInv,0.00000001); // invert (J * J Transpose)
	cout << "invert J * J Transpose :" << endl;
	JacDotJacTransInv.printMatrix();

	JacTranspose.computeMatrixMul(&JacDotJacTransInv, &JacPesudo); //J+ = J Transpose * (invert (J * J Transpose))
	cout << "pesudo J :" << endl;
	JacPesudo.printMatrix();
	JacTranspose.computeMatrixMul(&delta_e, &delta_theta);

	//JacPesudo.computeMatrixMul(&delta_e, &delta_theta); // invertmatrix seems sonthing wrong
	cout << "theta: " << endl;
	delta_theta.printMatrix();
	for (int j = 0; j < 3;j++) {// apply rotation to each joint
		int idx = 21 - j;
		Quatf rotation_quat;
		rotation_quat.clear();
		rotation_quat = multiply(rotation((float)(delta_theta.getValue(j * 3 + 0, 0) *dynamic_scaler* M_PI / 180.0f), make_vector(1.0f, 0.0f, 0.0f)), rotation_quat);
		rotation_quat = multiply(rotation((float)(delta_theta.getValue(j * 3 + 1, 0)  *dynamic_scaler* M_PI / 180.0f), make_vector(0.0f, 1.0f, 0.0f)), rotation_quat);
		rotation_quat = multiply(rotation((float)(delta_theta.getValue(j * 3 + 2, 0)  *dynamic_scaler* M_PI / 180.0f), make_vector(0.0f, 0.0f, 1.0f)), rotation_quat);
		rotation_quat = normalize(rotation_quat);
		Quatf curRot = current_pose.bone_orientations[idx];
		Quatf result = multiply(curRot, rotation_quat);

		current_pose.bone_orientations[idx] = result;
	}
	float new_distance = length(ballPos - wb.tips[21]);
		
		if (previous_error != -1) {// dynamic scaler to help converge
			if (previous_error < new_distance) {
				if (dynamic_scaler > 0.01) {
					dynamic_scaler / 7;
				}
			}
			else{
				if (dynamic_scaler < 10) {
					dynamic_scaler *= 7;
				}
			}
		}

		previous_error = distance;

		
	current_active_pose = current_pose;
}
void BrowseMode::handle_event(SDL_Event const &event) {
	//movement of the ball
	if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_UP) {
		vertical_movement += 1;
	}
	if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_DOWN) {
		vertical_movement -= 1;
	}
	if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_LEFT) {
		horizontal_movement += 1;
	}
	if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_RIGHT) {
		horizontal_movement -= 1;
	}
	if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_w) {
		height += 1;
	}
	if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_s) {
		height -= 1;
	}

	if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_c) {//using ccd to calculte IK
		enable_ccd = true;
		enable_jacobian = false;
	}
	if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_j) {//using jacobian to calculate IK
		enable_ccd = false;
		enable_jacobian = true;
	}
	if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_r) {//stop IK stimulate
		enable_ccd = false;
		enable_jacobian = false;
	}


	if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_TAB) {
		track = !track;
	}
	if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE) {
		quit_flag = true;
	}
	if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_PAGEUP) {
		current_motion += 1;
		if (current_motion >= Library::motion_count()) {
			current_motion = 0;
		}
		time = 0.0f;
	}
	if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_PAGEDOWN) {
		current_motion -= 1;
		if (current_motion >= Library::motion_count()) {
			current_motion = Library::motion_count() - 1;
		}
		time = 0.0f;
	}
	if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_SPACE) {
		if (play_speed == 1.0f) play_speed = 0.1f;
		else if (play_speed == 0.1f) play_speed = 0.0f;
		else play_speed = 1.0f;
	}
	if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_d) {
		for (unsigned int m = 0; m < Library::motion_count(); ++m) {
			Library::Motion const &motion = Library::motion(m);
			cout << "Dumpping '" << motion.filename + ".global" << "'" << endl;
			ofstream out((motion.filename + ".global").c_str());
			out << "\"position.x\", \"position.z\", \"position.yaw\", \"root.x\", \"root.y\", \"root.z\"";
			for (unsigned int b = 0; b < motion.skeleton->bones.size(); ++b) {
				out << ", ";
				out << '"' << motion.skeleton->bones[b].name << ".x" << '"';
				out << ", ";
				out << '"' << motion.skeleton->bones[b].name << ".y" << '"';
				out << ", ";
				out << '"' << motion.skeleton->bones[b].name << ".z" << '"';
			}
			out << endl;
			for (unsigned int f = 0; f < motion.frames(); ++f) {
				Character::WorldBones w;
				Character::Pose p;
				motion.get_local_pose(f, p);
				Character::get_world_bones(p, w);
				{
					Character::StateDelta delta;
					motion.get_delta(0, f, delta);
					out << delta.position.x << ", " << delta.position.z << ", " << delta.orientation << ", " << p.root_position.x << ", " << p.root_position.y << ", " << p.root_position.z;
				}
				for (unsigned int b = 0; b < w.tips.size(); ++b) {
					for (unsigned int c = 0; c < 3; ++c) {
						out << ", ";
						out << w.tips[b].c[c];
					}
				}
				out << endl;
			}
		}
	}

	//rotate view:
	if (event.type == SDL_MOUSEMOTION && (event.motion.state & SDL_BUTTON(SDL_BUTTON_LEFT))) {
		Vector3f vec = camera - target;
		float len = length(vec);
		float theta_yaw = atan2(vec.z, vec.x);
		float theta_pitch = atan2(vec.y, sqrt(vec.x * vec.x + vec.z * vec.z));
		theta_yaw += event.motion.xrel / float(Graphics::screen_x) / len * 100;
		theta_pitch += event.motion.yrel / float(Graphics::screen_x) / len * 100;
		if (theta_pitch > 0.4 * M_PI) theta_pitch = 0.4 * M_PI;
		if (theta_pitch <-0.4 * M_PI) theta_pitch =-0.4 * M_PI;

		camera = make_vector(cosf(theta_yaw)*cosf(theta_pitch),sinf(theta_pitch),sinf(theta_yaw)*cosf(theta_pitch)) * len + target;
	}
	//pan view:
	if (event.type == SDL_MOUSEMOTION && (event.motion.state & SDL_BUTTON(SDL_BUTTON_MIDDLE))) {
		Vector3f to = normalize(camera - target);
		Vector3f right = normalize(cross_product(to, make_vector< float >(0,1,0)));
		Vector3f up = -cross_product(to, right);
		float len = length(camera - target);
		camera += right * event.motion.xrel / float(Graphics::screen_x) * len;
		camera += up * event.motion.yrel / float(Graphics::screen_x) * len;
		target += right * event.motion.xrel / float(Graphics::screen_x) * len;
		target += up * event.motion.yrel / float(Graphics::screen_x) * len;
	}
	//zoom view:
	if (event.type == SDL_MOUSEMOTION && (event.motion.state & SDL_BUTTON(SDL_BUTTON_RIGHT))) {
		Vector3f vec = camera - target;
		float len = length(vec);
		len *= pow(2, event.motion.yrel / float(Graphics::screen_x) * 10);
		if (len < 1) len = 1;
		if (len > 100) len = 100;
		camera = normalize(camera - target) * len + target;
	}
}

inline bool exists (const std::string& name) 
{
	struct stat buffer;   
	return (stat (name.c_str(), &buffer) == 0); 
}

void BrowseMode::draw() {
const int FloorSize = 100;
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();

	gluPerspective(60.0, Graphics::aspect(), 0.1, 10.0 * FloorSize);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	{
		float y = camera.y;
		if (y < 0.1f) y = 0.1f;
		gluLookAt(camera.x, y, camera.z, target.x, target.y, target.z, 0, 1, 0);
	}


	for (unsigned int pass = 0; pass <= 2; ++pass) {
		static Vector4f l = make_vector(0.0f, float(FloorSize), 0.0f, 1.0f);
		static Vector4f amb = make_vector(0.1f, 0.1f, 0.1f, 1.0f);
		static Vector4f dif = make_vector(1.0f, 1.0f, 1.0f, 1.0f);
		static Vector4f zero = make_vector(0.0f, 0.0f, 0.0f, 0.0f);
		glLightfv(GL_LIGHT0, GL_AMBIENT, amb.c);
		glLightfv(GL_LIGHT0, GL_DIFFUSE, dif.c);
		glLightfv(GL_LIGHT0, GL_POSITION, l.c);
		if (pass == 0) { //reflections.
			glEnable(GL_DEPTH_TEST);
			glEnable(GL_LIGHTING);
			glEnable(GL_COLOR_MATERIAL);
			glEnable(GL_LIGHT0);
			l.y *= -1;
			glLightfv(GL_LIGHT0, GL_POSITION, l.c);
			l.y *= -1;
			glPushMatrix();
			glScaled(1.0f,-1.0f, 1.0f); //flip over the floor.
		} else if (pass == 1) {
			//shadows.
			glDisable(GL_DEPTH_TEST);
			glEnable(GL_STENCIL_TEST);
			glStencilFunc(GL_ALWAYS, 1, 0xff);
			glStencilOp(GL_REPLACE, GL_REPLACE, GL_REPLACE);
			glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
			glPushMatrix();
			double mat[16] = {
				l.y,	0,	0,	0, //x multiplies this
				0,	0,	0,	-1, //y multiplies this
				0,	0,	l.y,	0, //z multiplies this
				-l.x*l.y,	0,	-l.z*l.y,	l.y
			};
			glMultMatrixd(mat);
		} else if (pass == 2) {
			glEnable(GL_DEPTH_TEST);
			glEnable(GL_LIGHTING);
		}
		//state can be used to translate and rotate the character
		//to an arbitrary position, but we don't need that.
		Character::State null;
		null.clear();
		static bool cant_load_skin = false; //Huge hack: only check for skin file once.
		if(! exists("player.skin"))
		{
			cant_load_skin = true;
		}
		if (!skin.skin_buffer && !cant_load_skin) {
			load("player.skin", current_pose.skeleton, skin);
			if (!skin.skin_buffer) {
				cerr << "Looks like you don't have a player.skin file, so you get cylinders instead." << endl;
				cant_load_skin = true;
			}
		}
		if (skin.skin_buffer && pass == 2) {
			skin.calculate(current_pose, null);
			skin.draw();
		} else {
			Character::draw(current_pose, null, pass == 2, pass != 1);
			GLUquadricObj *pObj = gluNewQuadric();
			gluQuadricDrawStyle(pObj, GLU_FILL);
			glColor4f(1.0f, 1.0f, 0.1f, 1.0);

			// Create the sphere and move it back and to the right of the tube
			glPushMatrix();
			glTranslatef(ballPos.x,ballPos.y,ballPos.z);
			gluSphere(pObj, 0.5f, 25, 25);
			glPopMatrix();
		}
		if (pass == 0) { //cleanup post-reflections.
			glPopMatrix();
			glDisable(GL_LIGHTING);

			//overwrite anything sticking through the floor:
			glDepthFunc(GL_GEQUAL);
			glBegin(GL_QUADS);
			glColor3f(0,0,0);
			glVertex3f(-FloorSize,0,-FloorSize);
			glVertex3f(-FloorSize,0, FloorSize);
			glVertex3f( FloorSize,0, FloorSize);
			glVertex3f( FloorSize,0,-FloorSize);
			glEnd();
			glDepthFunc(GL_LESS);

			glDisable(GL_DEPTH_TEST);

		} else if (pass == 1) {
			glPopMatrix();

			//Now to deal with shadows.
			glEnable(GL_STENCIL_TEST);
			glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
			glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);

			//draw the floor:
			glEnable(GL_LIGHTING);
			glEnable(GL_BLEND);
			glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
			for (int floor_pass = 0; floor_pass < 2; ++floor_pass) {
				if (floor_pass == 0) {
					glLightfv(GL_LIGHT0, GL_AMBIENT, amb.c);
					glLightfv(GL_LIGHT0, GL_DIFFUSE, zero.c);
					glStencilFunc(GL_EQUAL, 1, 0xff);
				} else if (floor_pass == 1) {
					glLightfv(GL_LIGHT0, GL_AMBIENT, amb.c);
					glLightfv(GL_LIGHT0, GL_DIFFUSE, dif.c);
					glStencilFunc(GL_NOTEQUAL, 1, 0xff);
				}
				glNormal3f(0,1,0);
				glPushMatrix();
				glScalef(FloorSize / 10.0f, 1.0f, FloorSize / 10.0f);
				glBegin(GL_QUADS);
				for (int x = -10; x < 10; ++x) {
					for (int z = -10; z <= 10; ++z) {
						if ((x & 1) ^ (z & 1)) {
							glColor4f(0.5, 0.5, 0.5, 0.1);
						} else {
							glColor4f(0.9, 0.9, 0.9, 0.3);
						}
						glVertex3f(x,0,z);
						glVertex3f(x+1,0,z);
						glVertex3f(x+1,0,z+1);
						glVertex3f(x,0,z+1);
					}
				}
				glEnd();
				glPopMatrix();
			}
			glDisable(GL_BLEND);
			glDisable(GL_LIGHTING);
			glDisable(GL_STENCIL_TEST);
		} else if (pass == 2) {
			glDisable(GL_LIGHTING);
			glDisable(GL_DEPTH_TEST);
		}
	}
	glDisable(GL_DEPTH_TEST);

	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);

	//--- now some info overlay ---
	glPushMatrix();
	glLoadIdentity();
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glScaled(1.0 / Graphics::aspect(), 1.0, 1.0);
	glMatrixMode(GL_MODELVIEW);

	{
		Graphics::FontRef gentium = Graphics::get_font("gentium.txf");
		ostringstream info1, info2, info3, info4,info5;
		Library::Motion const &motion = Library::motion(current_motion);
		info1 << "Motion: " << motion.filename;
		info2 << "Skeleton: " << motion.skeleton->filename;
		info3 << "Frame " << (unsigned int)(time / motion.skeleton->timestep) << " of " << motion.frames() << " (" << 1.0f / motion.skeleton->timestep << " fps)";
		info4 << play_speed << "x speed";
		info5 << ballPos << "Ball Position";
		glEnable(GL_BLEND);
		glEnable(GL_TEXTURE_2D);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glColor3f(1.0f, 1.0f, 1.0f);
		Vector2f pos = make_vector(-(float)Graphics::aspect(), 1.0f);
		const float height = 0.07f;
		pos.y -= height;
		gentium.ref->draw(info1.str(), pos, height);
		pos.y -= height;
		gentium.ref->draw(info2.str(), pos, height);
		pos.y -= height;
		gentium.ref->draw(info3.str(), pos, height);
		pos.y -= height;
		gentium.ref->draw(info4.str(), pos, height);
		pos.y -= height;
		gentium.ref->draw(info5.str(), pos, height);
		glDisable(GL_TEXTURE_2D);
		glDisable(GL_BLEND);
	}

	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);

	Graphics::gl_errors("BrowseMode::draw");
}
