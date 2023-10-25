#include <Arduino_Neuton.h>
#include <ArduinoBLE.h>
#include <Arduino.h>
#include <Arduino_BHY2.h>
#include <Nicla_System.h>
#include <LibPrintf.h>

#include <assert.h>

// Define a custom service and characteristic UUID
BLEService customService("00001101-0000-1000-8000-00805f9b34fb");  // Custom service UUID
BLEStringCharacteristic customCharacteristic("00002101-0000-1000-8000-00805f9b34fb", BLERead | BLEBroadcast | BLENotify, 20);

//BLEService customService("1101"); // Custom service UUID
//BLEStringCharacteristic customCharacteristic("2101", BLERead | BLENotify, 20);

// C:\Users\Oriana\Documents\Arduino\libraries\Arduino_BHY2\src\BoschSensortec.h -> #define WORK_BUFFER_SIZE    64
// ///////////////////////////////////////////////////////////////////////////

#define LED_TOGGLE_FREQUENCY_HZ (5)
#define LED_TOGGLE_INTERVAL_MS (1000 / LED_TOGGLE_FREQUENCY_HZ)

#define DATA_FREQUENCY_HZ (100)  // how many times per second we read data -> doesn't work with a different number
#define DATA_FEED_INTERVAL_MS (1000 / DATA_FREQUENCY_HZ)
#define SENSOR_DATA_LEN (6U)
#define OUTPUT_CLASSES_NUM (5U)

#define RANDOM_CLASS (0U)
#define CLAPPING_CLASS (1U)
#define BRUSHING_TEETH_CLASS (2U)
#define WASHING_HANDS_CLASS (3U)
#define BRUSHING_HAIR_CLASS (4U)


#define PREDICTION_POSTPROCESSING_ENABLED 1

#define HOP_MS (200)
#define AVG_TIME_MS (400)
#define OUTPUTS_NUM_FOR_AVERAGING ((AVG_TIME_MS / HOP_MS) + (AVG_TIME_MS % HOP_MS ? 1 : 0))

//                                      RAND   CLAP   TEETH   HANDS   HAIR
#define THRESHOLDS 0.50, 0.91, 0.85, 0.92, 0.85

// ///////////////////////////////////////////////////////////////////////////
// Function prototypes

static void ledIndication(neuton_u16_t predictedClass, unsigned long currentTime);
static void handlePrediction(neuton_u16_t predictedClass,
                             const neuton_output_t* probabilities,
                             neuton_u16_t probabilitiesNum);
static void printPredictedClass(neuton_u16_t predictedClass,
                                neuton_output_t probability,
                                neuton_u16_t probabilitiesNum);
static const char* getClassName(neuton_u16_t classIndex);
static void sendToPhone(const char* message);

#if PREDICTION_POSTPROCESSING_ENABLED
static void postprocessingInit(void);
static void postprocessingBufferRdyCallback(void* pWindow, neuton_u16_t windowSize,
                                            void* userCtx);
#endif

// ///////////////////////////////////////////////////////////////////////////
// Local variables

SensorXYZ accel(SENSOR_ID_ACC);
SensorXYZ gyro(SENSOR_ID_GYRO);
static unsigned long previousSensorDataTime;
static neuton_sliding_window_ctx_t probabilitiesSlidingWindow;
static neuton_u16_t lastPredictedClass = RANDOM_CLASS;

const char* lastSentMessage = NULL;

// ///////////////////////////////////////////////////////////////////////////

void setup() {

  // Initialize Serial Port for debug info
  Serial.begin(115200);
  while (!Serial)
    ;

  if (!BLE.begin()) {
    Serial.println("Failed to initialize BLE!");
    while (1)
      ;
  }
  BLE.setLocalName("ArduinoBLE");
  BLE.setAdvertisedService(customService);

  customService.addCharacteristic(customCharacteristic);
  BLE.addService(customService);

  // Start advertising
  BLE.advertise();
  Serial.println("advertising ...");

  // Initialize Nicla Board Support Package
  nicla::begin();
  nicla::leds.begin();

  // Initialize BHY Inertial Sensor Module
  BHY2.begin(NICLA_I2C);
  accel.begin();
  gyro.begin();

  previousSensorDataTime = millis();

  // Initialize Neuton NN library
  neuton_nn_setup();

  // Check Neuton model Input and Output parameters
  assert(SENSOR_DATA_LEN == neuton_nn_uniq_inputs_num());
  assert(OUTPUT_CLASSES_NUM == neuton_nn_model_outputs_num());

// If PREDICTION_POSTPROCESSING_ENABLED option is enabled,
// then all prediction will be stored OUTPUTS_NUM_FOR_AVERAGING times and averaged with using a sliding window,
// and based on the maximum mean probability, a decision will be made on the current class
#if PREDICTION_POSTPROCESSING_ENABLED
  postprocessingInit();
#endif
}

// ///////////////////////////////////////////////////////////////////////////

void loop() {
  BLEDevice central = BLE.central();

  if (central.discoverAttributes()) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());
  }

  //if (central) {     // if (central.discoverAttributes())
  //  Serial.print("Connected to central: ");
  //  Serial.println(central.address());

  if (central.connected()) {  // if
    // Update function should be continuously polled
    BHY2.update();

    auto currentTime = millis();

    // Update LED indication color based on current class of activity
    ledIndication(lastPredictedClass, currentTime);

    if (currentTime - previousSensorDataTime >= DATA_FEED_INTERVAL_MS) {
      previousSensorDataTime = currentTime;

      // Read & store sensor data into the feature vector for Neuton model processing,
      // the order of input features should be the same as it is used in training dataset
      neuton_input_t sensorData[SENSOR_DATA_LEN];

      sensorData[0] = accel.x();
      sensorData[1] = accel.y();
      sensorData[2] = accel.z();
      sensorData[3] = gyro.x();
      sensorData[4] = gyro.y();
      sensorData[5] = gyro.z();

      // Feed input sensor data to the Neuton preprocessing pipeline,
      // when data will be ready for feature extraction and inference
      // the output data pointer will be not NULL
      neuton_inference_input_t* input = neuton_nn_feed_inputs(sensorData, SENSOR_DATA_LEN);

      if (input != NULL) {
        neuton_u16_t predictedClass;
        const neuton_output_t* probabilities;

        // Run inference of the Neuton model, predicted class of activity will be defined,
        // number of output classes should be equal to number of classes in the model
        auto outputsNum = neuton_nn_run_inference(input, &predictedClass, &probabilities);

        if (outputsNum > 0) {
          // Handle prediction results with postprocessing or just print it
          handlePrediction(predictedClass, probabilities, outputsNum);
        }
      }
    }
  }
  //Serial.print("Disconnected from central: ");
  //Serial.println(central.address());
  //}
}

// ///////////////////////////////////////////////////////////////////////////

static void sendToPhone(const char* message) {
  if (lastSentMessage == message) {
    return;
  }

  // BLEDevice central = BLE.central();

  // if (central) {
  // Serial.println(central.address());
  // if (central.connected()) {
  customCharacteristic.writeValue(message);
  Serial.println("Changed message");
  lastSentMessage = message;
  // return;
  // }
  // }
}


static void ledIndication(neuton_u16_t predictedClass, unsigned long currentTime) {
  static auto lastToggleTime = millis();
  static auto previosLedColor = off;

  if ((currentTime - lastToggleTime) >= LED_TOGGLE_INTERVAL_MS) {
    auto color = off;

    if (previosLedColor == off) {
      switch (predictedClass) {
        case RANDOM_CLASS: color = blue; break;
        case CLAPPING_CLASS: color = green; break;
        case BRUSHING_TEETH_CLASS: color = magenta; break;
        case WASHING_HANDS_CLASS: color = yellow; break;
        case BRUSHING_HAIR_CLASS: color = cyan; break;
        default: break;
      }
    }

    // nicla::leds.setColor(color);

    previosLedColor = color;
    lastToggleTime = currentTime;
  }
}

// ///////////////////////////////////////////////////////////////////////////

static void handlePrediction(neuton_u16_t predictedClass,
                             const neuton_output_t* probabilities,
                             neuton_u16_t probabilitiesNum) {

#if PREDICTION_POSTPROCESSING_ENABLED
  // Feed sliding window of probabilities for averaging results
  neuton_sliding_window_feed(&probabilitiesSlidingWindow,
                             (void*)probabilities, probabilitiesNum);

#else
  // Show predicted results
  printPredictedClass(predictedClass, probabilities[predictedClass], probabilitiesNum);
#endif
}

// ///////////////////////////////////////////////////////////////////////////

static void printPredictedClass(neuton_u16_t predictedClass,
                                neuton_output_t probability,
                                neuton_u16_t probabilitiesNum) {
  // If PREDICTION_POSTPROCESSING_ENABLED check that the probability
  // of the current activity exceeds the threshold necessary to indicate the result
#if PREDICTION_POSTPROCESSING_ENABLED
  static const neuton_output_t PROBABILITITES_THRESHOLDS[] = { THRESHOLDS };

  if (probability < PROBABILITITES_THRESHOLDS[predictedClass])
    return;
#endif

  lastPredictedClass = predictedClass;
  printf("%lu -> %s (%3u%%)\n", previousSensorDataTime, getClassName(predictedClass),
         (uint32_t)(probability * 100.0));

  const char* predictionData = getClassName(predictedClass);
  sendToPhone(predictionData);
}

// ///////////////////////////////////////////////////////////////////////////

static const char* getClassName(neuton_u16_t classIndex) {
  switch (classIndex) {
    case RANDOM_CLASS: return "Random        ";
    case CLAPPING_CLASS: return "Clapping      ";
    case BRUSHING_TEETH_CLASS: return "Brushing Teeth";
    case WASHING_HANDS_CLASS: return "Washing Hands ";
    case BRUSHING_HAIR_CLASS: return "Brushing Hair ";
    default: return "Unknown       ";
  }
}

// ///////////////////////////////////////////////////////////////////////////

#if PREDICTION_POSTPROCESSING_ENABLED

static void postprocessingInit(void) {
  static constexpr size_t WINDOW_LEN = OUTPUT_CLASSES_NUM * OUTPUTS_NUM_FOR_AVERAGING;
  static neuton_output_t slidingWindowBuffer[WINDOW_LEN];

  auto slidingWindowHop = neuton_nn_model_outputs_num();
  auto slidingWindowSampleSize = sizeof(neuton_output_t);

  neuton_sliding_window_init(&probabilitiesSlidingWindow,
                             (void*)slidingWindowBuffer,
                             WINDOW_LEN,
                             slidingWindowSampleSize,
                             slidingWindowHop,
                             postprocessingBufferRdyCallback,
                             NULL);
}

// ///////////////////////////////////////////////////////////////////////////

static void postprocessingBufferRdyCallback(void* pWindow, neuton_u16_t windowSize,
                                            void* userCtx) {
  (void)userCtx;
  auto outputsNum = neuton_nn_model_outputs_num();
  uint16_t classIndex = 0;
  float maxProb = 0;

  for (size_t i = 0; i < outputsNum; i++) {
    const float* probWindow = (float*)pWindow + i;
    auto meanProb = neuton_dsp_mean_f32_s(probWindow, OUTPUTS_NUM_FOR_AVERAGING, outputsNum, NULL);

    if (meanProb > maxProb) {
      maxProb = meanProb;
      classIndex = i;
    }
  }

  printPredictedClass(classIndex, maxProb, outputsNum);
}

#endif  // #if PREDICTION_POSTPROCESSING_ENABLED \
        // ///////////////////////////////////////////////////////////////////////////