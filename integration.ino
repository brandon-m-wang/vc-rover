/*
 * integration.ino
 */

/********************************************************/
/********************************************************/
/***                                                  ***/
/*** Constants and global variables from turning.ino. ***/
/***                                                  ***/
/********************************************************/
/********************************************************/

// Change pins here if you did not use the default pins!
#define LEFT_MOTOR                  P2_0
#define LEFT_ENCODER                P6_2
#define RIGHT_MOTOR                 P1_5
#define RIGHT_ENCODER               P6_3

#define SAMPLING_INTERVAL           100
int sample_lens[4] = {0};

// Operation modes
#define MODE_LISTEN                 0
#define MODE_DRIVE                  1
int timer_mode = MODE_LISTEN;

#define DRIVE_FAR                   0
#define DRIVE_LEFT                  1
#define DRIVE_CLOSE                 2
#define DRIVE_RIGHT                 3

#define JOLT_STEPS                  2

boolean loop_mode = MODE_DRIVE;
int drive_mode = 0;

int step_num = 0;
volatile boolean do_loop = 0; // timer signal to increment timestep

typedef struct encoder {
  int pin;
  int pos;
  bool level;
  int avg;
} encoder_t;

encoder_t left_encoder = {LEFT_ENCODER, 0, LOW, 0};
encoder_t right_encoder = {RIGHT_ENCODER, 0, LOW, 0};

/*---------------------------*/
/*      CODE BLOCK CON1      */
/*      From turning.ino     */
/*---------------------------*/
float theta_left = 0.1719;
float theta_right = 0.1549;
float beta_left = -1.244;
float beta_right = -3.10;
float v_star = 28.9;

// PWM inputs to jolt the car straight
int left_jolt = 250;
int right_jolt = 250;

// Control gains
float f_left = .25;
float f_right = .5;

/*---------------------------*/
/*      CODE BLOCK CON2      */
/*      From turning.ino     */
/*---------------------------*/

float driveStraight_left(float delta) {
  return (1/theta_left)*(v_star - f_left*delta + beta_left);
}

float driveStraight_right(float delta) {
  return (1/theta_right)*(v_star + (f_right*delta) + beta_right);
}


/*---------------------------*/
/*      CODE BLOCK CON3      */
/*      From turning.ino     */
/*---------------------------*/
float delta_ss = 0;

/*---------------------------*/
/*      CODE BLOCK CON4      */
/*      From turning.ino     */
/*---------------------------*/

#define CAR_WIDTH                   15.0 // in cm
#define TURN_RADIUS                 65.0 // in cm - 6 feet diameter = 3 tiles in 125 Cory
// #define TURN_RADIUS                 60 // in cm - 4 feet diameter = 2 tiles in 125 Cory

int run_times[4] = {7000, 5000, 2500, 5000}; // {DRIVE_FAR, DRIVE_LEFT, DRIVE_CLOSE, DRIVE_RIGHT}

float delta_reference(int i) {
  // YOUR CODE HERE
  // Remember to divide the v* value you use in this function by 5 because of sampling interval differences!
  if (drive_mode == DRIVE_RIGHT) { // Return a NEGATIVE expression
    return -CAR_WIDTH*i*v_star/(TURN_RADIUS*5);
  }
  else if (drive_mode == DRIVE_LEFT) { // Return a POSITIVE expression
    return CAR_WIDTH*i*v_star/(TURN_RADIUS*5);
  }
  else { // DRIVE_STRAIGHT
    return 0;
  }
}

/*---------------------------*/
/*      CODE BLOCK CON5      */
/*      From turning.ino     */
/*---------------------------*/
#define INFINITY                    (3.4e+38)
#define STRAIGHT_RADIUS             INFINITY

float straight_correction(int i) {
  // YOUR CODE HERE
  return 0;
}

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/

/*********************************************************/
/*********************************************************/
/***                                                   ***/
/*** Constants and glboal variables from classify.ino. ***/
/***                                                   ***/
/*********************************************************/
/*********************************************************/

// Change pin here if you did not use the default pin!
#define MIC_INPUT                   P6_0

#define SIZE                        3200
#define SIZE_AFTER_FILTER           200
#define ADC_TIMER_MS                0.35

/*---------------------------*/
/*      CODE BLOCK PCA1      */
/*     From classify.ino     */
/*---------------------------*/

// Enveloping and threshold constants
#define SNIPPET_SIZE                  80
#define PRELENGTH                     5
#define THRESHOLD                     0.5

#define EUCLIDEAN_THRESHOLD         0.05
#define LOUDNESS_THRESHOLD          500

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/


/*---------------------------*/
/*      CODE BLOCK PCA2      */
/*     From classify.ino     */
/*---------------------------*/

float pca_vec1[80] = {0.004891504898684299, 0.005286877313804228, 0.009007710694881266, 0.012520856421620263, -0.031673957539772625, -0.10358978043623572, -0.1593833399717529, -0.18521582642861142, -0.17806281268710986, -0.18802800005629525, -0.19430541319752798, -0.16527771430865038, -0.1692882592750843, -0.17989334281002609, -0.16536151542523747, -0.16292472217479947, -0.16722429704187172, -0.16604237622005227, -0.17012048288680395, -0.15900522427052433, -0.1577719155794444, -0.14500629516388064, -0.1521863590074593, -0.1484999148690051, -0.14164880451623318, -0.11883363051461228, -0.11272175302110353, -0.1019509231435702, -0.08157511320124038, -0.051071697938368577, -0.027928685160843042, -0.0027305448788315654, 0.017835756478078595, 0.041248401162169, 0.06933865014271788, 0.08432697742116453, 0.10361322921534721, 0.1249568701476146, 0.13376435268261824, 0.14946356191976842, 0.1676231418297621, 0.1699807164594346, 0.1649596243143757, 0.17388426005005758, 0.17129786571943315, 0.15620509726417028, 0.15367414251292877, 0.14080584852721215, 0.12452218494641053, 0.11959352789837842, 0.10482313706587554, 0.08419300020620835, 0.07287040851385324, 0.05499779082342463, 0.04298663738240561, 0.04345493011618712, 0.02139306065549872, 0.02603812275226742, 0.029629820787901676, 0.02136819451137068, 0.015100005392407518, 0.022068905157475154, 0.01851074373898042, 0.022000510606637477, 0.03143777975513902, 0.024726020893263062, 0.032714338480636776, 0.04209575489023624, 0.04124937727944364, 0.050302831489138326, 0.05447214581685313, 0.05101846171165068, 0.05810282414651763, 0.07395922332459741, 0.06791783432930339, 0.06872017862915422, 0.083301284536951, 0.07439231222739018, 0.07354438036141588, 0.08113152812213155};
float pca_vec2[80] = {-0.03629936544027929, -0.05020964500880387, -0.07200219971076471, -0.07845746143412657, -0.05442828624949771, -0.04275426968269398, -0.014185051853899943, -0.0029492451081032786, -0.03145373187826375, -0.014585185651655584, -0.008911943102214515, -0.037824798101707365, -0.026753781198865893, -0.024561377528202585, -0.014949431364949828, -0.004378771024499292, -0.018978878230561103, -0.020352509450946157, -0.0004321628905117824, -0.015853187747263398, 0.0024253554585111138, 0.010765544756814623, 0.005335080177750187, 0.05964129711344916, 0.06175797716946295, 0.07191481003247063, 0.08069893280043049, 0.07376649610962613, 0.07881208184050137, 0.0799444144288588, 0.07553105043231888, 0.033127356066830246, 0.04000943445235812, 0.0437189937013487, 0.005957799779261002, -0.0019926816193914223, -0.005689028117156273, -0.038888215409787166, -0.03519987693534808, -0.020303090492735546, -0.022752251915166007, -0.00027067063471447746, 0.008531611807697584, 0.02776921576329353, 0.05697368501408572, 0.09178474657263229, 0.11227629931160632, 0.13342525714494105, 0.1589518347840132, 0.16806935508823265, 0.17788260815434323, 0.18530639736410012, 0.18553865125577704, 0.17615244132232985, 0.18149842351546025, 0.17190439300615257, 0.1530775269947258, 0.15603130595696094, 0.1417153298444777, 0.1148639285340787, 0.10057083844230162, 0.08646203250046115, 0.05553554451760543, 0.03442953623786773, -0.00763090101991676, -0.039446889578065, -0.08194524655561972, -0.1247093719565462, -0.1608040429049009, -0.1795909451838114, -0.20048393032141162, -0.19986734072891627, -0.22128945473361603, -0.23243121711833717, -0.21343645078309303, -0.21803330917886807, -0.2282958344794609, -0.19595772282457052, -0.20670051027594624, -0.19611732202794732};
float pca_vec3[80] = {0.0004527739939320816, -0.004448101361578711, -0.02297004129817569, -0.021471731779959837, -0.03693540051309853, -0.14150658546623224, -0.12929526478372289, -0.13365598848438742, -0.13146573223508703, -0.06661285559936374, -0.07410153098452073, -0.0606269735967537, -0.0688706916383928, -0.03622185481762274, -0.05696696029578148, -0.023951942096778037, 0.008241765203426373, -0.0039454335917437616, -0.0021710582361507223, 0.013903267692617216, 0.0024290722728671043, 0.018347106159019928, 0.03714730787702577, 0.04856696305657563, 0.09699024260897246, 0.12872963444685306, 0.1621481305468599, 0.18269210224245325, 0.22763764109627327, 0.22907160802003076, 0.2585250233010356, 0.29071705855335744, 0.2750671050435367, 0.2646018606476615, 0.2602209909554728, 0.22551579952171677, 0.21093585377885934, 0.17031264058487355, 0.13844484062278498, 0.0841571152551153, 0.05104092087351201, 0.02079681583402911, -0.0078004381106349645, -0.0388732943342511, -0.06465441905909097, -0.06315257290067536, -0.07737383785373479, -0.088021789122151, -0.08446803242085701, -0.08566600442619884, -0.1098113414694301, -0.10113559477021182, -0.10300233363730064, -0.11770592770390838, -0.12827834705523952, -0.09780030115286754, -0.11158547388219844, -0.10228543280652602, -0.09267853176786421, -0.07279715564215374, -0.08188327447721513, -0.06720005146110043, -0.05655447238212193, -0.0453845711875349, -0.035553575539428756, -0.028385237688353427, -0.02485913942349402, -0.02668888720926708, -0.021811375317641414, -0.01776010591999096, -0.023749946186120793, -0.01692338127327579, -0.033844291700076065, -0.05179589699851878, -0.0375096419141441, -0.04561442349009661, -0.06040958930348612, -0.04998451234532592, -0.0544074770479035, -0.06406481442912179};
float projected_mean_vec[3] = {-0.02965802579489301, -0.0018614882131845357, 0.009099760583473367};
float centroid1[3] = {0.011457892032644594, 0.030779781982805926, 0.005227462468456067};
float centroid2[3] = {0.006655898230007901, -0.025595978141521456, 0.017025051465388175};
float centroid3[3] = {0.03112496167486089, -0.019474337768542228, -0.007672078624081688};
float centroid4[3] = {-0.04923875193751339, 0.014290533927257754, -0.014580435309762533};
float* centroids[4] = {
  (float *) &centroid1, (float *) &centroid2,
  (float *) &centroid3, (float *) &centroid4
};


/*---------------------------*/
/*---------------------------*/
/*---------------------------*/

float result[SNIPPET_SIZE] = {0};
float proj1 = 0;
float proj2 = 0;
float proj3 = 0;

// Data array and index pointer
int re[SIZE] = {0};
volatile int re_pointer = 0;

/*---------------------------------------------------*/
/*---------------------------------------------------*/
/*---------------------------------------------------*/

/*---------------------------*/
/*       Norm functions      */
/*---------------------------*/

// Compute the L2 norm of (dim1, dim2) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        centroid: size-2 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm(float dim1, float dim2, float* centroid) {
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2));
}

// Compute the L2 norm of (dim1, dim2, dim3) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        dim3: 3rd dimension coordinate
//        centroid: size-3 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm3(float dim1, float dim2, float dim3, float* centroid) {
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2) + pow(dim3-centroid[2],2));
}

void setup(void) {
  Serial.begin(38400);

  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(LEFT_ENCODER, INPUT);
  pinMode(RIGHT_MOTOR, OUTPUT);
  pinMode(RIGHT_ENCODER, INPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(MIC_INPUT, INPUT);

  for (int j = 0; j < 4; j++) {
    sample_lens[j] = run_times[j] / SAMPLING_INTERVAL;
  }

  write_pwm(0, 0);
  delay(2000); // Wait 2 seconds to put down car
  reset_blinker();
  start_listen_mode();
}

void loop(void) {
  check_encoders();
  if (timer_mode == MODE_LISTEN && re_pointer == SIZE){
    // Stop motor
    write_pwm(0, 0);
    digitalWrite(RED_LED, LOW);

    // if enveloped data is above some preset value
    if (envelope(re, result)) {

      // Reset projection result variables declared above
      proj1 = 0;
      proj2 = 0;
      proj3 = 0;

      /*---------------------------*/
      /*      CODE BLOCK PCA3      */
      /*     From classify.ino     */
      /*     with more changes     */
      /*---------------------------*/

      // Project 'result' onto the principal components
      // YOUR CODE HERE
      for (int i = 0; i < SNIPPET_SIZE; i++) {
          proj1 += result[i]*pca_vec1[i];
          proj2 += result[i]*pca_vec2[i];
          proj3 += result[i]*pca_vec3[i];
      }

      // Demean the projection
      proj1 -= projected_mean_vec[0];
      proj2 -= projected_mean_vec[1];
      proj3 -= projected_mean_vec[2];


      // Classification
      // Use the function l2_norm3 defined above
      // jth centroid: centroids[j]
      // YOUR CODE HERE
      float best_dist = 999999;
      int best_index = -1;
      // YOUR CODE HERE
      for (int i = 0; i < 4; i++) {
        float curr_dist = l2_norm3(proj1, proj2, proj3, centroids[i]);
        if (curr_dist < best_dist) {
          best_dist = curr_dist;
          best_index = i;
        }
      }


      // Check against EUCLIDEAN_THRESHOLD and execute identified command
      // YOUR CODE HERE
      String signals[4] = {"KIWI", "PINEAPPLE", "AVOCADO", "WISCONSIN"};
      if (best_dist >= EUCLIDEAN_THRESHOLD) {
        Serial.println("THRESHOLD NOT SATISFIED");
      } else {
        Serial.println(best_index);
        if (signals[best_index] == "AVOCADO") {
            drive_mode = DRIVE_CLOSE;
            start_drive_mode();
        }
        else if (signals[best_index] == "KIWI") {
            drive_mode = DRIVE_LEFT; //LEFT
            start_drive_mode();
        }
        else if (signals[best_index] == "PINEAPPLE") {
            drive_mode = DRIVE_RIGHT;
            start_drive_mode();
        }
        else if (signals[best_index] == "WISCONSIN") {
            drive_mode = DRIVE_FAR;
            start_drive_mode();
        } 
      }

      /*---------------------------*/
      /*---------------------------*/
      /*---------------------------*/
    }

    delay(2000);
    re_pointer = 0; // start recording from beginning if we don't start driving
  }

  else if (loop_mode == MODE_DRIVE && do_loop) {
    if (step_num < JOLT_STEPS) {
      write_pwm(left_jolt, right_jolt);
    }
    else {

      // Save positions because _left_position and _right_position
      // can change in the middle of one loop.
      int left_position = left_encoder.pos;
      int right_position = right_encoder.pos;

      /*---------------------------*/
      /*      CODE BLOCK CON0      */
      /*---------------------------*/

      float delta = left_position - right_position + delta_ss;
      delta = delta + delta_reference(step_num) + straight_correction(step_num);

      // Drive straight using feedback
      // Compute the needed pwm values for each wheel using delta and v_star
      int left_cur_pwm = driveStraight_left(delta);
      int right_cur_pwm = driveStraight_right(delta);
      write_pwm(left_cur_pwm, right_cur_pwm);

      /*---------------------------*/
      /*---------------------------*/
      /*---------------------------*/
    }

    // Counter for how many times loop is executed since entering DRIVE MODE
    step_num++;

    if (step_num == sample_lens[drive_mode]) {
      // Completely stop and go back to listen MODE after 3 seconds
      start_listen_mode();
    }

    do_loop = 0;
  }
}

// Enveloping function with thresholding and normalizing,
// returns snippet of interest (containing speech)
bool envelope(int* data, float* data_out) {
  int32_t avg = 0;
  float maximum = 0;
  int32_t total = 0;
  int block;

  // Apply enveloping filter while finding maximum value
  for (block = 0; block < SIZE_AFTER_FILTER; block++) {
    avg = 0;
    for (int j = 0; j < 16; j++) {
      avg += data[j+block*16];
    }
    avg = avg >> 4;
    data[block] = abs(data[block*16] - avg);
    for (int j = 1; j < 16; j++) {
      data[block] += abs(data[j+block*16] - avg);
    }
    if (data[block] > maximum) {
      maximum = data[block];
    }
  }

  // If not loud enough, return false
  if (maximum < LOUDNESS_THRESHOLD) {
    return false;
  }

  // Determine threshold
  float thres = THRESHOLD * maximum;

  // Figure out when interesting snippet starts and write to data_out
  block = PRELENGTH;
  while (data[block++] < thres);
  if (block > SIZE_AFTER_FILTER - SNIPPET_SIZE) {
    block = SIZE_AFTER_FILTER - SNIPPET_SIZE;
  }
  for (int j = 0; j < SNIPPET_SIZE; j++) {
    data_out[j] = data[block-PRELENGTH+j];
    total += data_out[j];
  }

  // Normalize data_out
  for (int j = 0; j < SNIPPET_SIZE; j++) {
    data_out[j] = data_out[j] / total;
  }

  return true;
}

/*---------------------------*/
/*     Helper functions      */
/*---------------------------*/

void write_pwm(int pwm_left, int pwm_right) {
  analogWrite(LEFT_MOTOR, (int) min(max(0, pwm_left), 255));
  analogWrite(RIGHT_MOTOR, (int) min(max(0, pwm_right), 255));
}

void reset_blinker(void) {
  digitalWrite(RED_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, LOW);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(GREEN_LED, LOW);
}

void start_listen_mode(void) {
  re_pointer = 0;
  write_pwm(0, 0);
  delay(3000); // 3 seconds buffer for mic cap settling
  timer_mode = MODE_LISTEN;
  setTimer(MODE_LISTEN);
}

void start_drive_mode(void) {
  timer_mode = MODE_DRIVE;
  step_num = 0;
  left_encoder.pos = 0;
  right_encoder.pos = 0;
  setTimer(MODE_DRIVE);
}

/*---------------------------*/
/*    Interrupt functions    */
/*---------------------------*/

#define AVG_DECAY_RATE              0.3
#define LOW_THRESH                  ((int) (0.1*4096))
#define HIGH_THRESH                 ((int) (0.4*4096))

void check_encoder(encoder_t* enc) {
  int new_val = analogRead(enc->pin);
  enc->avg = (int) (AVG_DECAY_RATE*enc->avg + (1 - AVG_DECAY_RATE)*new_val);
  if ((enc->level == LOW && HIGH_THRESH < enc->avg) ||
      (enc->level == HIGH && enc->avg < LOW_THRESH)) {
    enc->pos++;
    enc->level = !enc->level;
  }
}

void check_encoders(void) {
  check_encoder(&left_encoder);
  check_encoder(&right_encoder);
}

// Set timer for timestep; use B0 to free up all other PWM ports
void setTimer(boolean mode) {
  if (mode == MODE_LISTEN) {
    // Set the timer based on 25MHz clock
    TB0CCR0 = (unsigned int) (25000*ADC_TIMER_MS);
    TB0CCTL0 = CCIE;
    __bis_SR_register(GIE);
    TB0CTL = TASSEL_2 + MC_1 + TACLR + ID_0;
  }
  else if (mode == MODE_DRIVE) {
    TB0CCR0 = (unsigned int) (32.768*SAMPLING_INTERVAL); // set the timer based on 32kHz clock
    TB0CCTL0 = CCIE; // enable interrupts for Timer B
    __bis_SR_register(GIE);
    TB0CTL = TASSEL_1 + MC_1 + TACLR + ID_0;
  }
  timer_mode = mode;
}

// ISR for timestep
#pragma vector=TIMER0_B0_VECTOR    // Timer B ISR
__interrupt void Timer0_B0_ISR(void) {
  if (timer_mode == MODE_LISTEN) {
    if (re_pointer < SIZE) {
      digitalWrite(RED_LED, HIGH);
      re[re_pointer] = (analogRead(MIC_INPUT) >> 4) - 128;
      re_pointer += 1;
    }
  }
  else if (timer_mode == MODE_DRIVE) {
    do_loop = 1;
  }
}
