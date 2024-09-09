// PF CONVERGENCE TEST
#include <Arduino.h>

// Define the number of particles
const int NUM_PARTICLES = 5000;

// Define a structure for particles
struct Particle {
  float x;
  float y;
  float weight;
};

// Initialise particles array
Particle particles[NUM_PARTICLES];

// Initialise resampling particle array
Particle new_particles[NUM_PARTICLES];

// Assume a constant theta for simplicity
const float theta = PI / 4;  // 45 degrees
const float sensor_noise_std_dev = 200;
float measured_x, measured_y;
float likelihood_coeff;

void updatePFWeights(float measurement_x, float measurement_y) {
  float total_weight = 0.0;

  for (int i = 0; i < NUM_PARTICLES; i++) {
    // Calculate the likelihood of the actual measurement given the particle's state
    float likelihood = measurementLikelihood(particles[i].x, particles[i].y, measurement_x, measurement_y);

    // Update the particle's weight
    particles[i].weight *= likelihood;

    for (int i; i < NUM_PARTICLES; i++) {
      if (particles[i].weight == 0) {
        Serial.print("Particle ");
        Serial.print(i);
        Serial.println(" has weight = 0");
      }
    }

    // Sum the total weight for normalization
    total_weight += particles[i].weight;
  }

  // Normalise the weights
  for (int i = 0; i < NUM_PARTICLES; i++) {
    particles[i].weight /= total_weight;
  }
}

float measurementLikelihood(float particle_x, float particle_y, float measurement_x, float measurement_y) {
  // Calculate Euclidean distance between particle state and actual measurement
  // float distance = sqrt(pow(particle_x - measurement_x, 2) + pow(particle_y - measurement_y, 2));

  // // Calculate the likelihood using a Gaussian distribution
  // float likelihood = (likelihood_coeff) * exp(-pow(distance, 2) / (2.0 * sensor_noise_std_dev * sensor_noise_std_dev));

  // More efficient computation for the likelihood 
  float likelihood = (likelihood_coeff) * exp(-(pow(particle_x - measurement_x, 2) + pow(particle_y - measurement_y, 2)) / (2.0 * sensor_noise_std_dev * sensor_noise_std_dev));

  return likelihood;
}

void systematicResamplePF(void) {
  float weight_interval = 1.0f / NUM_PARTICLES;
  float random_start = random(0, 1000) / 1000.0f * weight_interval;
  float cumulative_weight = particles[0].weight;
  int particle_index = 0;

  for (int j = 0; j < NUM_PARTICLES; j++) {
    float step_threshold = random_start + j * weight_interval;
    while (step_threshold > cumulative_weight && particle_index < NUM_PARTICLES - 1) {
      particle_index++;
      cumulative_weight += particles[particle_index].weight;
    }
    new_particles[j] = particles[particle_index];
  }

  for (int j = 0; j < NUM_PARTICLES; j++) {
    particles[j] = new_particles[j];
  }
}

// void estimatePF

void setup() {
  Serial.begin(115200);
  // Initialize particles
  for (int i = 0; i < NUM_PARTICLES; i++) {
    particles[i].x = random(0, 10);             // Random initial x position
    particles[i].y = random(0, 10);             // Random initial y position
    particles[i].weight = 1.0 / NUM_PARTICLES;  // Equal weight initially
  }

  likelihood_coeff = 1.0 / sqrt(2.0 * PI * sensor_noise_std_dev * sensor_noise_std_dev);
}

void loop() {
  unsigned long startTime = millis();

  // Prediction Step
  for (int i = 0; i < NUM_PARTICLES; i++) {
    particles[i].x += cos(theta);  // Update x position
    particles[i].y += sin(theta);  // Update y position
  }

  // Take measurement
  measured_x = random(0, 10);
  measured_y = random(0, 10);

  // Serial.println("Before weight update:");
  // for (int i = 0; i < 5; i++) {  // Just as an example, printing the first 5 particles
  //   Serial.print("Particle ");
  //   Serial.print(i);
  //   Serial.print(" - X: ");
  //   Serial.print(particles[i].x);
  //   Serial.print(", Y: ");
  //   Serial.print(particles[i].y);
  //   Serial.print(", Weight: ");
  //   Serial.println(particles[i].weight);
  // }

  updatePFWeights(measured_x, measured_y);

  Serial.println("Likelihoods:");
  for (int i = 0; i < 5; i++) {
    float likelihood = measurementLikelihood(particles[i].x, particles[i].y, measured_x, measured_y);
    Serial.print("Particle ");
    Serial.print(i);
    Serial.print(" Likelihood: ");
    Serial.println(likelihood);
  }

  Serial.println("After weight update and normalization:");
  for (int i = 0; i < 5; i++) {
    Serial.print("Particle ");
    Serial.print(i);
    Serial.print(" - Weight: ");
    Serial.println(particles[i].weight);
  }


  // // Here you would normally incorporate measurement update,
  // // but for this example, let's assume we just normalize weights after "measurement"
  // // For simplicity, all weights remain equal in this example
  // float totalWeight = 0;
  // for (int i = 0; i < NUM_PARTICLES; i++) {
  //   totalWeight += particles[i].weight;
  // }
  // for (int i = 0; i < NUM_PARTICLES; i++) {
  //   particles[i].weight /= totalWeight;  // Normalize weights
  // }

  // // Resampling Step (Simple Resampling Method for Illustration)
  // Particle newParticles[NUM_PARTICLES];
  // for (int i = 0; i < NUM_PARTICLES; i++) {
  //   int index = random(0, NUM_PARTICLES);
  //   // Swap particles[i] with particles[index]
  //   Particle temp = particles[i];
  //   particles[i] = particles[index];
  //   particles[index] = temp;
  // }

  systematicResamplePF();

  Serial.println("After resampling:");
  for (int i = 0; i < 5; i++) {
    Serial.print("Particle ");
    Serial.print(i);
    Serial.print(" - X: ");
    Serial.print(particles[i].x);
    Serial.print(", Y: ");
    Serial.print(particles[i].y);
    Serial.print(", Weight: ");
    Serial.println(particles[i].weight);
  }

  // Estimate Position
  float estimatedX = 0;
  float estimatedY = 0;
  for (int i = 0; i < NUM_PARTICLES; i++) {
    estimatedX += particles[i].x * particles[i].weight;
    estimatedY += particles[i].y * particles[i].weight;
  }

  // Print the estimated position
  Serial.print("Estimated Position -> X: ");
  Serial.print(estimatedX);
  Serial.print(", Y: ");
  Serial.println(estimatedY);

  // Compute and print the computation time
  unsigned long endTime = millis();
  Serial.print("Computation Time: ");
  Serial.print(endTime - startTime);
  Serial.println(" ms");

  // Wait for 1.5 seconds before next iteration
  delay(1500);
}
