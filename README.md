# Stroke Rehabilitation Monitoring Bracelet

This repository contains the code and resources for a stroke rehabilitation monitoring bracelet. The bracelet, equipped with the Nicla Sense ME Arduino board, captures and transmits hand activity data of stroke survivors. This data is then visualized in a web app, enabling doctors to monitor patient progress remotely in real-time.

# Contents
* Arduino/: Arduino sketch for the Nicla Sense ME board and the library of our trained model.
* WebApp/: Source code for the web application used by healthcare professionals.
* MobileApplication/: Source code for the mobile application used by patients.
* Node-red/: Source code for the node-red flow designed as a middleware for the web-page and mobile device.

# Features
* Real-Time Data Transmission: Using BLE, the bracelet transmits hand activity data to a mobile app, which then uploads it to a cloud service.
* Data Visualization: The web app provides physicians with access to real-time data, historical analytics, and personalized patient reports.
* User-Friendly Interface: Designed for ease of use by healthcare professionals.

# Setup and Installation
Arduino Bracelet
* Requirements: Arduino IDE, Nicla Sense Me board.
* Installation: Open the Arduino IDE, navigate to Arduino/, and upload the sketch to the Nicla Sense ME board.

Web Application
* Requirements: A modern web browser.
* Installation: Navigate to the WebApp/ directory and follow the setup instructions detailed in the WebApp/README.md.

# Usage
* For Patients: Wear the bracelet during daily activities for continuous monitoring.
* For Physicians: Log in to the web app to view and analyze patient data.

# Contributing
We welcome contributions to this project. 
