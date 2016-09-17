//float delta[ABC];
float cartesian_position[ABC] = { 0 };
//float endstop_adj[ABC] = { 0 };
float diagrod_adj[ABC] = { 0 };
float tower_adj[6] = { 0 };
//float delta_radius;
//float delta_diagonal_rod;
float delta_tmp[ABC] = { 0.0 };
//float delta_tower1_x, delta_tower1_y, delta_tower2_x, delta_tower2_y, delta_tower3_x, delta_tower3_y;
//float base_max_pos[ABC] = { X_MAX_POS, Y_MAX_POS, Z_MAX_POS };
//float base_home_pos[ABC] = { X_HOME_POS, Y_HOME_POS, Z_HOME_POS };
//float max_length[ABC] = { X_MAX_LENGTH, Y_MAX_LENGTH, Z_MAX_LENGTH };
//float delta_diagonal_rod_1, delta_diagonal_rod_2, delta_diagonal_rod_3;
//float delta_clip_start_height = Z_MAX_POS;
float delta_safe_distance_from_top();
//float delta_segments_per_second;

const float bed_radius = DELTA_PROBEABLE_RADIUS;
//const float z_probe_deploy_start_location[] = Z_PROBE_DEPLOY_START_LOCATION;
//const float z_probe_deploy_end_location[] = Z_PROBE_DEPLOY_END_LOCATION;
//const float z_probe_retract_start_location[] = Z_PROBE_RETRACT_START_LOCATION;
//const float z_probe_retract_end_location[] = Z_PROBE_RETRACT_END_LOCATION;
int   delta_grid_spacing[2] = { 0, 0 };
float bed_level[AUTO_BED_LEVELING_GRID_POINTS][AUTO_BED_LEVELING_GRID_POINTS];
float ac_prec = AUTOCALIBRATION_PRECISION;
float bed_level_c, bed_level_x, bed_level_y, bed_level_z,
bed_level_ox, bed_level_oy, bed_level_oz, bed_safe_z;
float adj_t1_Radius = 0;
float adj_t2_Radius = 0;
float adj_t3_Radius = 0;
float probe_bed(float x, float y);
void  adj_tower_delta(uint8_t tower);
void  adj_tower_radius(uint8_t tower);
void  home_delta_axis();
void  calibration_report();
void  bed_probe_all();
void  adjust_delta(float cartesian[ABC]);
void  adj_endstops();
void  reset_bed_level();
bool  delta_leveling_in_progress = false;


void home_delta_axis() {

	// Init the current position of all carriages to 0,0,0
	memset(current_position, 0, sizeof(current_position));
	sync_plan_position();

	// Move all carriages up together until the first endstop is hit.
	//current_position[X_AXIS] = current_position[Y_AXIS] = current_position[Z_AXIS] = 3.0 * max_length[Z_AXIS];
	feedrate_mm_s = homing_feedrate_mm_s[X_AXIS];
	line_to_current_position();
	stepper.synchronize();
	endstops.hit_on_purpose(); // clear endstop hit flags

	memset(current_position, 0, sizeof(current_position));

	// At least one carriage has reached the top.
	// Now back off and re-home each carriage separately.
	HOMEAXIS(A);
	HOMEAXIS(B);
	HOMEAXIS(C);

	// Set all carriages to their home positions
	// Do this here all at once for Delta, because
	// XYZ isn't ABC. Applying this per-tower would
	// give the impression that they are the same.
	LOOP_XYZ(i) set_axis_is_at_home((AxisEnum)i);

	SYNC_PLAN_POSITION_KINEMATIC();

	//if (DEBUGGING(INFO)) //DEBUG_INFO_POS("(DELTA)", current_position);

	// move to a height where we can use the full xy-area
	do_blocking_move_to_z(delta_clip_start_height);

}

/*
void set_delta_constants() {
	max_length[Z_AXIS] = soft_endstop_max[Z_AXIS] - Z_MIN_POS;
	base_max_pos[Z_AXIS] = soft_endstop_max[Z_AXIS];
	base_home_pos[Z_AXIS] = soft_endstop_max[Z_AXIS];

	delta_diagonal_rod_1 = sq(delta_diagonal_rod + diagrod_adj[0]);
	delta_diagonal_rod_2 = sq(delta_diagonal_rod + diagrod_adj[1]);
	delta_diagonal_rod_3 = sq(delta_diagonal_rod + diagrod_adj[2]);

	// Effective X/Y positions of the three vertical towers.
	delta_tower1_x = (delta_radius + tower_adj[3]) * cos((210 + tower_adj[0]) * M_PI / 180); // front left tower
	delta_tower1_y = (delta_radius + tower_adj[3]) * sin((210 + tower_adj[0]) * M_PI / 180);
	delta_tower2_x = (delta_radius + tower_adj[4]) * cos((330 + tower_adj[1]) * M_PI / 180); // front right tower
	delta_tower2_y = (delta_radius + tower_adj[4]) * sin((330 + tower_adj[1]) * M_PI / 180);
	delta_tower3_x = (delta_radius + tower_adj[5]) * cos((90 + tower_adj[2]) * M_PI / 180);  // back middle tower
	delta_tower3_y = (delta_radius + tower_adj[5]) * sin((90 + tower_adj[2]) * M_PI / 180);
}*/

/*
void inverse_kinematics(const float in_cartesian[ABC]) {

	const float cartesian[ABC] = {
		RAW_X_POSITION(in_cartesian[X_AXIS]),
		RAW_Y_POSITION(in_cartesian[Y_AXIS]),
		RAW_Z_POSITION(in_cartesian[Z_AXIS])
	};

	delta[TOWER_1] = sqrt(delta_diagonal_rod_1
		- sq(delta_tower1_x - cartesian[X_AXIS])
		- sq(delta_tower1_y - cartesian[Y_AXIS])
		) + cartesian[Z_AXIS];
	delta[TOWER_2] = sqrt(delta_diagonal_rod_2
		- sq(delta_tower2_x - cartesian[X_AXIS])
		- sq(delta_tower2_y - cartesian[Y_AXIS])
		) + cartesian[Z_AXIS];
	delta[TOWER_3] = sqrt(delta_diagonal_rod_3
		- sq(delta_tower3_x - cartesian[X_AXIS])
		- sq(delta_tower3_y - cartesian[Y_AXIS])
		) + cartesian[Z_AXIS];
}*/

/*

float delta_safe_distance_from_top() {
	float cartesian[ABC] = {
		LOGICAL_X_POSITION(0),
		LOGICAL_Y_POSITION(0),
		LOGICAL_Z_POSITION(0)
	};
	inverse_kinematics(cartesian);
	float distance = delta[TOWER_3];
	cartesian[Y_AXIS] = LOGICAL_Y_POSITION(DELTA_PRINTABLE_RADIUS);
	inverse_kinematics(cartesian);
	return abs(distance - delta[TOWER_3]);
}

void forward_kinematics_DELTA(float z1, float z2, float z3) {
	//As discussed in Wikipedia "Trilateration"
	//we are establishing a new coordinate
	//system in the plane of the three carriage points.
	//This system will have the origin at tower1 and
	//tower2 is on the x axis. tower3 is in the X-Y
	//plane with a Z component of zero. We will define unit
	//vectors in this coordinate system in our original
	//coordinate system. Then when we calculate the
	//Xnew, Ynew and Znew values, we can translate back into
	//the original system by moving along those unit vectors
	//by the corresponding values.
	// https://en.wikipedia.org/wiki/Trilateration

	// Variable names matched to Marlin, c-version
	// and avoiding a vector library
	// by Andreas Hardtung 2016-06-7
	// based on a Java function from
	// "Delta Robot Kinematics by Steve Graves" V3

	// Result is in cartesian_position[].

	//Create a vector in old coordinates along x axis of new coordinate
	float p12[3] = { delta_tower2_x - delta_tower1_x, delta_tower2_y - delta_tower1_y, z2 - z1 };

	//Get the Magnitude of vector.
	float d = sqrt(p12[0] * p12[0] + p12[1] * p12[1] + p12[2] * p12[2]);

	//Create unit vector by dividing by magnitude.
	float ex[3] = { p12[0] / d, p12[1] / d, p12[2] / d };

	//Now find vector from the origin of the new system to the third point.
	float p13[3] = { delta_tower3_x - delta_tower1_x, delta_tower3_y - delta_tower1_y, z3 - z1 };

	//Now use dot product to find the component of this vector on the X axis.
	float i = ex[0] * p13[0] + ex[1] * p13[1] + ex[2] * p13[2];

	//Now create a vector along the x axis that represents the x component of p13.
	float iex[3] = { ex[0] * i,  ex[1] * i,  ex[2] * i };

	//Now subtract the X component away from the original vector leaving only the Y component. We use the
	//variable that will be the unit vector after we scale it.
	float ey[3] = { p13[0] - iex[0], p13[1] - iex[1], p13[2] - iex[2] };

	//The magnitude of Y component
	float j = sqrt(sq(ey[0]) + sq(ey[1]) + sq(ey[2]));

	//Now make vector a unit vector
	ey[0] /= j; ey[1] /= j;  ey[2] /= j;

	//The cross product of the unit x and y is the unit z
	//float[] ez = vectorCrossProd(ex, ey);
	float ez[3] = { ex[1] * ey[2] - ex[2] * ey[1], ex[2] * ey[0] - ex[0] * ey[2], ex[0] * ey[1] - ex[1] * ey[0] };

	//Now we have the d, i and j values defined in Wikipedia.
	//We can plug them into the equations defined in
	//Wikipedia for Xnew, Ynew and Znew
	float Xnew = (delta_diagonal_rod_1 - delta_diagonal_rod_2 + d*d) / (d * 2);
	float Ynew = ((delta_diagonal_rod_1 - delta_diagonal_rod_3 + i*i + j*j) / 2 - i*Xnew) / j;
	float Znew = sqrt(delta_diagonal_rod_1 - Xnew*Xnew - Ynew*Ynew);

	//Now we can start from the origin in the old coords and
	//add vectors in the old coords that represent the
	//Xnew, Ynew and Znew to find the point in the old system
	cartesian_position[X_AXIS] = delta_tower1_x + ex[0] * Xnew + ey[0] * Ynew - ez[0] * Znew;
	cartesian_position[Y_AXIS] = delta_tower1_y + ex[1] * Xnew + ey[1] * Ynew - ez[1] * Znew;
	cartesian_position[Z_AXIS] = z1 + ex[2] * Xnew + ey[2] * Ynew - ez[2] * Znew;
};

void forward_kinematics_DELTA(float point[ABC]) {
	forward_kinematics_DELTA(point[X_AXIS], point[Y_AXIS], point[Z_AXIS]);
}*/

void set_cartesian_from_steppers() {
	forward_kinematics_DELTA(stepper.get_axis_position_mm(X_AXIS),
		stepper.get_axis_position_mm(Y_AXIS),
		stepper.get_axis_position_mm(Z_AXIS));
}

#if ENABLED(AUTO_BED_LEVELING_FEATURE)

bool Equal_AB(const float A, const float B, const float prec = ac_prec) {
	if (abs(abs(A) - abs(B)) <= prec) return true;
	return false;
}

/**
* All DELTA leveling in the MK4duo uses NONLINEAR_BED_LEVELING
*/
/*
static void extrapolate_one_point(int x, int y, int xdir, int ydir) {
	if (bed_level[x][y] != 0.0) {
		return;  // Don't overwrite good values.
	}
	float a = 2 * bed_level[x + xdir][y] - bed_level[x + xdir * 2][y];  // Left to right.
	float b = 2 * bed_level[x][y + ydir] - bed_level[x][y + ydir * 2];  // Front to back.
	float c = 2 * bed_level[x + xdir][y + ydir] - bed_level[x + xdir * 2][y + ydir * 2];  // Diagonal.
	float median = c;  // Median is robust (ignores outliers).
	if (a < b) {
		if (b < c) median = b;
		if (c < a) median = a;
	}
	else {  // b <= a
		if (c < b) median = b;
		if (a < c) median = a;
	}
	bed_level[x][y] = median;
}*/

/**
* Fill in the unprobed points (corners of circular print surface)
* using linear extrapolation, away from the center.
*/
/*
static void extrapolate_unprobed_bed_level() {
	int half = (AUTO_BED_LEVELING_GRID_POINTS - 1) / 2;
	for (int y = 0; y <= half; y++) {
		for (int x = 0; x <= half; x++) {
			if (x + y < 3) continue;
			extrapolate_one_point(half - x, half - y, x > 1 ? +1 : 0, y > 1 ? +1 : 0);
			extrapolate_one_point(half + x, half - y, x > 1 ? -1 : 0, y > 1 ? +1 : 0);
			extrapolate_one_point(half - x, half + y, x > 1 ? +1 : 0, y > 1 ? -1 : 0);
			extrapolate_one_point(half + x, half + y, x > 1 ? -1 : 0, y > 1 ? -1 : 0);
		}
	}
}*/

/**
* Print calibration results for plotting or manual frame adjustment.
*/
/*
void print_bed_level() {
	for (int y = 0; y < AUTO_BED_LEVELING_GRID_POINTS; y++) {
		for (int x = 0; x < AUTO_BED_LEVELING_GRID_POINTS; x++) {
			if (bed_level[x][y] >= 0) //SERIAL_M(" ");
			SERIAL_V(bed_level[x][y]);
			SERIAL_C(' ');
		}
		SERIAL_E;
	}
}*/

/**
* Reset calibration results to zero.
*/
/*
void reset_bed_level() {
	if (DEBUGGING(INFO)) SERIAL_LM(INFO, "reset_bed_level");
	for (int y = 0; y < AUTO_BED_LEVELING_GRID_POINTS; y++) {
		for (int x = 0; x < AUTO_BED_LEVELING_GRID_POINTS; x++) {
			bed_level[x][y] = 0.0;
		}
	}
}*/

/**
* Probe bed height at position (x,y), returns the measured z value
*/
float probe_bed(float x, float y) {
	if (DEBUGGING(INFO)) {
		//SERIAL_SMV(INFO, ">>> probe_bed(", x);
		////SERIAL_MV(", ", y);
		//SERIAL_EM(")");
		//DEBUG_INFO_POS("", current_position);
	}

	float old_feedrate_mm_s = feedrate_mm_s;

	float Dx = x - (X_PROBE_OFFSET_FROM_EXTRUDER);
	NOLESS(Dx, X_MIN_POS);
	NOMORE(Dx, X_MAX_POS);
	float Dy = y - (Y_PROBE_OFFSET_FROM_EXTRUDER);
	NOLESS(Dy, Y_MIN_POS);
	NOMORE(Dy, Y_MAX_POS);

	if (DEBUGGING(INFO)) {
		//SERIAL_SMV(INFO, "do_blocking_move_to_xy(", Dx);
		////SERIAL_MV(", ", Dy);
		//SERIAL_EM(")");
	}

	// this also updates current_position
	feedrate_mm_s = XY_PROBE_FEEDRATE_MM_S;
	do_blocking_move_to_xy(Dx, Dy);

	float probe_z = run_z_probe() + zprobe_zoffset;

	if (DEBUGGING(INFO)) {
		//SERIAL_SM(INFO, "Bed probe heights: ");
		//if (probe_z >= 0) //SERIAL_M(" ");
		//SERIAL_EV(probe_z, 4);
	}

	// Move Z up to the bed_safe_z
	bed_safe_z = current_position[Z_AXIS] + Z_RAISE_BETWEEN_PROBINGS;
	do_probe_raise(bed_safe_z);

	feedrate_mm_s = old_feedrate_mm_s;

	return probe_z;
}

void bed_probe_all() {
	// Initial throwaway probe.. used to stabilize probe
	bed_level_c = probe_bed(0.0, 0.0);

	// Probe all bed positions & store carriage positions
	bed_level_z = probe_bed(0.0, bed_radius);
	bed_level_oy = probe_bed(-SIN_60 * bed_radius, COS_60 * bed_radius);
	bed_level_x = probe_bed(-SIN_60 * bed_radius, -COS_60 * bed_radius);
	bed_level_oz = probe_bed(0.0, -bed_radius);
	bed_level_y = probe_bed(SIN_60 * bed_radius, -COS_60 * bed_radius);
	bed_level_ox = probe_bed(SIN_60 * bed_radius, COS_60 * bed_radius);
	bed_level_c = probe_bed(0.0, 0.0);
}

void apply_endstop_adjustment(float x_endstop, float y_endstop, float z_endstop) {
	float saved_endstop_adj[ABC] = { 0 };
	memcpy(saved_endstop_adj, endstop_adj, sizeof(saved_endstop_adj));
	endstop_adj[X_AXIS] += x_endstop;
	endstop_adj[Y_AXIS] += y_endstop;
	endstop_adj[Z_AXIS] += z_endstop;

	inverse_kinematics(current_position);
	planner.set_position_mm(delta[TOWER_1] - (endstop_adj[TOWER_1] - saved_endstop_adj[TOWER_1]), delta[TOWER_2] - (endstop_adj[TOWER_2] - saved_endstop_adj[TOWER_2]), delta[TOWER_3] - (endstop_adj[TOWER_3] - saved_endstop_adj[TOWER_3]), current_position[E_AXIS]);
	stepper.synchronize();
}

void adj_endstops() {
	boolean x_done = false;
	boolean y_done = false;
	boolean z_done = false;
	float prv_bed_level_x, prv_bed_level_y, prv_bed_level_z;

	do {
		bed_level_z = probe_bed(0.0, bed_radius);
		bed_level_x = probe_bed(-SIN_60 * bed_radius, -COS_60 * bed_radius);
		bed_level_y = probe_bed(SIN_60 * bed_radius, -COS_60 * bed_radius);

		apply_endstop_adjustment(bed_level_x, bed_level_y, bed_level_z);

		////SERIAL_MV("x:", bed_level_x, 4);
		////SERIAL_MV(" (adj:", endstop_adj[0], 4);
		////SERIAL_MV(") y:", bed_level_y, 4);
		////SERIAL_MV(" (adj:", endstop_adj[1], 4);
		////SERIAL_MV(") z:", bed_level_z, 4);
		////SERIAL_MV(" (adj:", endstop_adj[2], 4);
		//SERIAL_EM(")");

		if ((bed_level_x >= -ac_prec) and (bed_level_x <= ac_prec)) {
			x_done = true;
			//SERIAL_M("X=OK ");
		}
		else {
			x_done = false;
			//SERIAL_M("X=ERROR ");
		}

		if ((bed_level_y >= -ac_prec) and (bed_level_y <= ac_prec)) {
			y_done = true;
			//SERIAL_M("Y=OK ");
		}
		else {
			y_done = false;
			//SERIAL_M("Y=ERROR ");
		}

		if ((bed_level_z >= -ac_prec) and (bed_level_z <= ac_prec)) {
			z_done = true;
			//SERIAL_EM("Z=OK");
		}
		else {
			z_done = false;
			//SERIAL_EM("Z=ERROR");
		}
	} while (((x_done == false) or (y_done == false) or (z_done == false)));

	float high_endstop = MAX3(endstop_adj[TOWER_1], endstop_adj[TOWER_2], endstop_adj[TOWER_3]);

	if (DEBUGGING(INFO)) {
		SERIAL_LMV(INFO, "High endstop:", high_endstop, 4);
	}

	if (high_endstop > 0) {
		////SERIAL_EMV("Reducing Build height by ", high_endstop);
		for (uint8_t i = 0; i < ABC; i++) {
			endstop_adj[i] -= high_endstop;
		}
		soft_endstop_max[Z_AXIS] -= high_endstop;
	}

	/*
	else if (high_endstop < 0) {
	////SERIAL_EMV("Increment Build height by ", abs(high_endstop));
	for(uint8_t i = 0; i < 3; i++) {
	endstop_adj[i] -= high_endstop;
	}
	soft_endstop_max[Z_AXIS] -= high_endstop;
	}
	*/

	set_delta_constants();
}

int fix_tower_errors() {
	boolean t1_err, t2_err, t3_err,
		xy_equal, xz_equal, yz_equal;
	float saved_tower_adj[6];
	uint8_t err_tower = 0;
	//float low_diff, high_diff, x_diff, y_diff, z_diff, xy_diff, yz_diff, xz_diff, low_opp, high_opp;

	for (uint8_t i = 0; i < 6; i++) saved_tower_adj[i] = tower_adj[i];

	x_diff = abs(bed_level_x - bed_level_ox);
	y_diff = abs(bed_level_y - bed_level_oy);
	z_diff = abs(bed_level_z - bed_level_oz);
	high_diff = MAX3(x_diff, y_diff, z_diff);

	if (x_diff <= ac_prec) t1_err = false; else t1_err = true;
	if (y_diff <= ac_prec) t2_err = false; else t2_err = true;
	if (z_diff <= ac_prec) t3_err = false; else t3_err = true;

	////SERIAL_MV("x_diff:", x_diff, 5);
	////SERIAL_MV(" y_diff:", y_diff, 5);
	////SERIAL_MV(" z_diff:", z_diff, 5);
	////SERIAL_EMV(" high_diff:", high_diff, 5);

	// Are all errors equal? (within defined precision)
	xy_equal = false;
	xz_equal = false;
	yz_equal = false;
	if (abs(x_diff - y_diff) <= ac_prec) xy_equal = true;
	if (abs(x_diff - z_diff) <= ac_prec) xz_equal = true;
	if (abs(y_diff - z_diff) <= ac_prec) yz_equal = true;

	//SERIAL_M("xy_equal = ");
	if (xy_equal == true) //SERIAL_EM("true"); else //SERIAL_EM("false");
	//SERIAL_M("xz_equal = ");
	if (xz_equal == true) //SERIAL_EM("true"); else //SERIAL_EM("false");
	//SERIAL_M("yz_equal = ");
	if (yz_equal == true) //SERIAL_EM("true"); else //SERIAL_EM("false");

	low_opp = MIN3(bed_level_ox, bed_level_oy, bed_level_oz);
	high_opp = MAX3(bed_level_ox, bed_level_oy, bed_level_oz);

	////SERIAL_EMV("Opp Range = ", high_opp - low_opp, 5);

	if (high_opp - low_opp  < ac_prec) {
		//SERIAL_EM("Opposite Points within Limits - Adjustment not required");
		t1_err = false;
		t2_err = false;
		t3_err = false;
	}

	// All Towers have errors
	if ((t1_err == true) and (t2_err == true) and (t3_err == true)) {
		if ((xy_equal == false) or (xz_equal == false) or (yz_equal == false)) {
			// Errors not equal .. select the tower that needs to be adjusted
			if (high_diff == x_diff) err_tower = 1;
			if (high_diff == y_diff) err_tower = 2;
			if (high_diff == z_diff) err_tower = 3;
			////SERIAL_MV("Tower ", err_tower);
			//SERIAL_EM(" has largest error");
		}
		if ((xy_equal == true) and (xz_equal == true) and (yz_equal == true)) {
			//SERIAL_EM("All Towers Errors Equal");
			t1_err = false;
			t2_err = false;
			t3_err = false;
		}
	}

	/*
	// Two tower errors
	if ((t1_err == true) and (t2_err == true) and (t3_err == false)) {
	if (high_diff == x_diff) err_tower = 1;
	else err_tower = 2;
	}
	else if ((t1_err == true) and (t2_err == false) and (t3_err == true)) {
	if (high_diff == x_diff) err_tower = 1;
	else err_tower = 3;
	}
	else if ((t1_err == false) and (t2_err == true) and (t3_err == true)) {
	if (high_diff == y_diff) err_tower = 2;
	else err_tower = 3;
	}
	*/

	// Single tower error
	if ((t1_err == true) and (t2_err == false) and (t3_err == false)) err_tower = 1;
	if ((t1_err == false) and (t2_err == true) and (t3_err == false)) err_tower = 2;
	if ((t1_err == false) and (t2_err == false) and (t3_err == true)) err_tower = 3;

	//SERIAL_M("t1:");
	if (t1_err == true) //SERIAL_M("Err"); else //SERIAL_M("OK");
	//SERIAL_M(" t2:");
	if (t2_err == true) //SERIAL_M("Err"); else //SERIAL_M("OK");
	//SERIAL_M(" t3:");
	if (t3_err == true) //SERIAL_M("Err"); else //SERIAL_M("OK");
	SERIAL_E;

	if (err_tower == 0) {
		//SERIAL_EM("Tower geometry OK");
	}
	else {
		////SERIAL_MV("Tower", int(err_tower));
		//SERIAL_EM(" Error: Adjusting");
		adj_tower_radius(err_tower);
	}

	// Set return value to indicate if anything has been changed (0 = no change)
	int retval = 0;
	for (uint8_t i = 0; i < 6; i++) if (saved_tower_adj[i] != tower_adj[i]) retval++;
	return retval;
}

bool adj_deltaradius() {
	boolean adj_done;
	int adj_attempts;
	float adj_dRadius, adjdone_vector;

	bed_level_c = probe_bed(0.0, 0.0);

	if ((bed_level_c >= -ac_prec) and (bed_level_c <= ac_prec)) {
		//SERIAL_EM("Delta Radius OK");
		return false;
	}
	else {
		//SERIAL_EM("Adjusting Delta Radius");
		////SERIAL_EMV("Bed level center = ", bed_level_c);

		// set initial direction and magnitude for delta radius adjustment
		adj_attempts = 0;
		adj_dRadius = 0;
		adjdone_vector = 0.01;

		do {
			delta_radius += adj_dRadius;
			set_delta_constants();
			adj_done = false;

			adj_endstops();
			bed_level_c = probe_bed(0.0, 0.0);

			// Set inital adjustment value if it is currently 0
			if (adj_dRadius == 0) {
				if (bed_level_c > 0) adj_dRadius = -0.2;
				if (bed_level_c < 0) adj_dRadius = 0.2;
			}

			// Adjustment complete?
			if ((bed_level_c >= -ac_prec) and (bed_level_c <= ac_prec)) {
				//Done to within acprec .. but done within adjdone_vector? 
				if ((bed_level_c >= -adjdone_vector) and (bed_level_c <= adjdone_vector))
					adj_done = true;
				else {
					adj_attempts++;
					if (adj_attempts > 3) {
						adjdone_vector += 0.01;
						adj_attempts = 0;
					}
				}
			}

			// Show progress
			////SERIAL_MV(" c:", bed_level_c, 4);
			////SERIAL_MV(" delta radius:", delta_radius, 4);
			////SERIAL_MV(" prec:", adjdone_vector, 3);
			////SERIAL_MV(" tries:", adj_attempts);
			//SERIAL_M(" done:");
			if (adj_done == true) //SERIAL_EM("true");
			else //SERIAL_EM("false");

			// Overshot target? .. reverse and scale down adjustment
			if (((bed_level_c < 0) and (adj_dRadius < 0)) or ((bed_level_c > 0) and (adj_dRadius > 0))) adj_dRadius = -(adj_dRadius / 2);

		} while (adj_done == false);

		return true;
	}
}

void adj_tower_radius(uint8_t tower) {
	boolean adj_done;
	float adj_tRadius = 0.0, bed_level, bed_level_o;

	do {
		tower_adj[tower + 2] += adj_tRadius;
		set_delta_constants();
		adj_done = false;

		if (tower == 1) {
			// Bedlevel_x
			bed_level = probe_bed(-SIN_60 * bed_radius, -COS_60 * bed_radius);
			// Bedlevel_ox
			bed_level_o = probe_bed(SIN_60 * bed_radius, COS_60 * bed_radius);
		}
		if (tower == 2) {
			// Bedlevel_y
			bed_level = probe_bed(SIN_60 * bed_radius, -COS_60 * bed_radius);
			// Bedlevel_oy
			bed_level_o = probe_bed(-SIN_60 * bed_radius, COS_60 * bed_radius);
		}
		if (tower == 3) {
			// Bedlevel_z
			bed_level = probe_bed(0.0, bed_radius);
			// Bedlevel_oz
			bed_level_o = probe_bed(0.0, -bed_radius);
		}

		// Set inital adjustment value if it is currently 0
		if (adj_tRadius == 0) {
			if (bed_level_o < bed_level) adj_tRadius = -1;
			if (bed_level_o > bed_level) adj_tRadius = 1;
		}

		// Overshot target? .. reverse and scale down adjustment
		if (((bed_level_o < bed_level) and (adj_tRadius > 0)) or ((bed_level_o > bed_level) and (adj_tRadius < 0))) adj_tRadius = -(adj_tRadius / 2);

		// Adjustment complete?
		if ((bed_level_o > bed_level - 0.015) and (bed_level_o < bed_level + 0.015)) adj_done = true;

		// Show progress
		////SERIAL_MV("tower:", bed_level, 4);
		////SERIAL_MV(" opptower:", bed_level_o, 4);
		////SERIAL_MV(" tower radius adj:", tower_adj[tower + 2], 4);
		//SERIAL_M(" done:");
		if (adj_done == true) //SERIAL_EM("true");
		else //SERIAL_EM("false");

		if (adj_done == false) adj_endstops();

	} while (adj_done == false);
}

void adj_tower_delta(uint8_t tower) {
	float adj_val = 0;
	float adj_mag = 0.2;
	float adj_prv;

	do {
		tower_adj[tower - 1] += adj_val;
		set_delta_constants();

		if ((tower == 1) or (tower == 3)) bed_level_oy = probe_bed(-SIN_60 * bed_radius, COS_60 * bed_radius);
		if ((tower == 1) or (tower == 2)) bed_level_oz = probe_bed(0.0, -bed_radius);
		if ((tower == 2) or (tower == 3)) bed_level_ox = probe_bed(SIN_60 * bed_radius, COS_60 * bed_radius);

		adj_prv = adj_val;
		adj_val = 0;

		if (tower == 1) {
			if (bed_level_oy < bed_level_oz) adj_val = adj_mag;
			if (bed_level_oy > bed_level_oz) adj_val = -adj_mag;
		}

		if (tower == 2) {
			if (bed_level_oz < bed_level_ox) adj_val = adj_mag;
			if (bed_level_oz > bed_level_ox) adj_val = -adj_mag;
		}

		if (tower == 3) {
			if (bed_level_ox < bed_level_oy) adj_val = adj_mag;
			if (bed_level_ox > bed_level_oy) adj_val = -adj_mag;
		}

		if ((adj_val > 0) and (adj_prv < 0)) {
			adj_mag = adj_mag / 2;
			adj_val = adj_mag;
		}

		if ((adj_val < 0) and (adj_prv > 0)) {
			adj_mag = adj_mag / 2;
			adj_val = -adj_mag;
		}

		// Show Adjustments made
		if (tower == 1) {
			////SERIAL_MV("oy:", bed_level_oy, 4);
			////SERIAL_MV(" oz:", bed_level_oz, 4);
		}

		if (tower == 2) {
			////SERIAL_MV("ox:", bed_level_ox, 4);
			////SERIAL_MV(" oz:", bed_level_oz, 4);
		}

		if (tower == 3) {
			////SERIAL_MV("ox:", bed_level_ox, 4);
			////SERIAL_MV(" oy:", bed_level_oy, 4);
		}

		////SERIAL_EMV(" tower delta adj:", adj_val, 5);
	} while (adj_val != 0);
}

float adj_diagrod_length() {
	float adj_val = 0;
	float adj_mag = 0.2;
	float adj_prv, target;
	float prev_diag_rod = delta_diagonal_rod;

	do {
		delta_diagonal_rod += adj_val;
		set_delta_constants();

		bed_level_oy = probe_bed(-SIN_60 * bed_radius, COS_60 * bed_radius);
		bed_level_oz = probe_bed(0.0, -bed_radius);
		bed_level_ox = probe_bed(SIN_60 * bed_radius, COS_60 * bed_radius);
		bed_level_c = probe_bed(0.0, 0.0);

		target = (bed_level_ox + bed_level_oy + bed_level_oz) / 3;
		adj_prv = adj_val;
		adj_val = 0;

		if (bed_level_c - 0.01 < target) adj_val = -adj_mag;
		if (bed_level_c + 0.01 > target) adj_val = adj_mag;

		if (((adj_val > 0) and (adj_prv < 0)) or ((adj_val < 0) and (adj_prv > 0))) {
			adj_val = adj_val / 2;
			adj_mag = adj_mag / 2;
		}

		if ((bed_level_c - 0.01 < target) and (bed_level_c + 0.01 > target)) adj_val = 0;

		// If adj magnatude is very small.. quit adjusting
		if ((abs(adj_val) < 0.001) and (adj_val != 0)) adj_val = 0;

		////SERIAL_MV("target:", target, 4);
		////SERIAL_MV(" c:", bed_level_c, 4);
		////SERIAL_EMV(" adj:", adj_val, 5);
	} while (adj_val != 0);

	return (delta_diagonal_rod - prev_diag_rod);
}

void calibrate_print_surface() {
	//float probe_bed_z, probe_z, probe_h, probe_l;
	//int probe_count, auto_bed_leveling_grid_points = AUTO_BED_LEVELING_GRID_POINTS;

	int left_probe_bed_position = LEFT_PROBE_BED_POSITION,
		right_probe_bed_position = RIGHT_PROBE_BED_POSITION,
		front_probe_bed_position = FRONT_PROBE_BED_POSITION,
		back_probe_bed_position = BACK_PROBE_BED_POSITION;

	// probe at the points of a lattice grid
	const int xGridSpacing = (right_probe_bed_position - left_probe_bed_position) / (auto_bed_leveling_grid_points - 1),
		yGridSpacing = (back_probe_bed_position - front_probe_bed_position) / (auto_bed_leveling_grid_points - 1);

	delta_grid_spacing[X_AXIS] = xGridSpacing;
	delta_grid_spacing[Y_AXIS] = yGridSpacing;

	// First point
	bed_level_c = probe_bed(0.0, 0.0);

	bool zig = true;

	for (int yCount = 0; yCount < auto_bed_leveling_grid_points; yCount++) {
		double yProbe = front_probe_bed_position + yGridSpacing * yCount;
		int xStart, xStop, xInc;

		if (zig) {
			xStart = 0;
			xStop = auto_bed_leveling_grid_points;
			xInc = 1;
		}
		else {
			xStart = auto_bed_leveling_grid_points - 1;
			xStop = -1;
			xInc = -1;
		}

		zig = !zig;

		for (int xCount = xStart; xCount != xStop; xCount += xInc) {
			double xProbe = left_probe_bed_position + xGridSpacing * xCount;

			// Avoid probing the corners (outside the round or hexagon print surface) on a delta printer.
			float distance_from_center = sqrt(xProbe * xProbe + yProbe * yProbe);
			if (distance_from_center > DELTA_PROBEABLE_RADIUS) continue;

			bed_level[xCount][yCount] = probe_bed(xProbe, yProbe);

			idle();
		} // xProbe
	} // yProbe

	extrapolate_unprobed_bed_level();
	print_bed_level();
}

void calibration_report() {
	// Display Report
	//SERIAL_EM("| \tZ-Tower\t\t\tEndstop Offsets");

	//SERIAL_M("| \t");
	//if (bed_level_z >= 0) //SERIAL_M(" ");
	////SERIAL_MV("", bed_level_z, 4);
	////SERIAL_MV("\t\t\tX:", endstop_adj[0], 4);
	////SERIAL_MV(" Y:", endstop_adj[1], 4);
	////SERIAL_EMV(" Z:", endstop_adj[2], 4);

	//SERIAL_M("| ");
	//if (bed_level_ox >= 0) //SERIAL_M(" ");
	////SERIAL_MV("", bed_level_ox, 4);
	//SERIAL_M("\t");
	//if (bed_level_oy >= 0) //SERIAL_M(" ");
	////SERIAL_MV("", bed_level_oy, 4);
	//SERIAL_EM("\t\tTower Offsets");

	//SERIAL_M("| \t");
	//if (bed_level_c >= 0) //SERIAL_M(" ");
	////SERIAL_MV("", bed_level_c, 4);
	////SERIAL_MV("\t\t\tA:", tower_adj[0]);
	////SERIAL_MV(" B:", tower_adj[1]);
	////SERIAL_EMV(" C:", tower_adj[2]);

	//SERIAL_M("| ");
	//if (bed_level_x >= 0) //SERIAL_M(" ");
	////SERIAL_MV("", bed_level_x, 4);
	//SERIAL_M("\t");
	//if (bed_level_y >= 0) //SERIAL_M(" ");
	////SERIAL_MV("", bed_level_y, 4);
	////SERIAL_MV("\t\tI:", tower_adj[3]);
	////SERIAL_MV(" J:", tower_adj[4]);
	////SERIAL_EMV(" K:", tower_adj[5]);

	//SERIAL_M("| \t");
	//if (bed_level_oz >= 0) //SERIAL_M(" ");
	////SERIAL_MV("", bed_level_oz, 4);
	////SERIAL_EMV("\t\t\tDelta Radius: ", delta_radius, 4);

	////SERIAL_EMV("| X-Tower\tY-Tower\t\tDiagonal Rod: ", delta_diagonal_rod, 4);
	//SERIAL_E;
}

/**
* Adjust print surface height by linear interpolation over the bed_level array.
*/
/*
void adjust_delta(float cartesian[ABC]) {
	if (delta_grid_spacing[X_AXIS] == 0 || delta_grid_spacing[Y_AXIS] == 0) return; // G29 not done!

	int half = (AUTO_BED_LEVELING_GRID_POINTS - 1) / 2;
	float h1 = 0.001 - half, h2 = half - 0.001,
		grid_x = max(h1, min(h2, RAW_X_POSITION(cartesian[X_AXIS]) / delta_grid_spacing[X_AXIS])),
		grid_y = max(h1, min(h2, RAW_Y_POSITION(cartesian[Y_AXIS]) / delta_grid_spacing[Y_AXIS]));
	int floor_x = floor(grid_x), floor_y = floor(grid_y);
	float ratio_x = grid_x - floor_x, ratio_y = grid_y - floor_y,
		z1 = bed_level[floor_x + half][floor_y + half],
		z2 = bed_level[floor_x + half][floor_y + half + 1],
		z3 = bed_level[floor_x + half + 1][floor_y + half],
		z4 = bed_level[floor_x + half + 1][floor_y + half + 1],
		left = (1 - ratio_y) * z1 + ratio_y * z2,
		right = (1 - ratio_y) * z3 + ratio_y * z4,
		offset = (1 - ratio_x) * left + ratio_x * right;

	delta[TOWER_1] += offset;
	delta[TOWER_2] += offset;
	delta[TOWER_3] += offset;

	if (DEBUGGING(ALL)) {
		//SERIAL_SMV(DEB, "grid_x=", grid_x);
		////SERIAL_MV(" grid_y=", grid_y);
		////SERIAL_MV(" floor_x=", floor_x);
		////SERIAL_MV(" floor_y=", floor_y);
		////SERIAL_MV(" ratio_x=", ratio_x);
		////SERIAL_MV(" ratio_y=", ratio_y);
		////SERIAL_MV(" z1=", z1);
		////SERIAL_MV(" z2=", z2);
		////SERIAL_MV(" z3=", z3);
		////SERIAL_MV(" z4=", z4);
		////SERIAL_MV(" left=", left);
		////SERIAL_MV(" right=", right);
		////SERIAL_EMV(" offset=", offset);
	}
}*/

#endif // AUTO_BED_LEVELING_FEATURE