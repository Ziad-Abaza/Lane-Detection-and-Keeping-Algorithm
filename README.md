# Lane Detection and Keeping Algorithm

This project implements a **Lane Detection and Keeping Algorithm** that processes video input and detects lane markings on the road. The algorithm uses computer vision techniques to extract lane lines, calculate the steering angle required for lane keeping, and display the heading line to visualize the steering direction.

## Features

- **Edge Detection**: Uses Canny edge detection to find edges in the frame.
- **Region of Interest**: Crops the image to focus only on the area relevant for lane detection.
- **Lane Line Detection**: Uses Hough Transform to detect lane line segments.
- **Lane Fitting**: Averages detected lines and fits polynomial curves to create smoother lane lines.
- **Steering Angle Calculation**: Computes the steering angle based on the position of detected lane lines.
- **Heading Line Display**: Visualizes the computed steering direction by drawing a heading line.
- **Real-Time Processing**: Processes video frames in real-time to detect and keep the car within lane boundaries.

## Project Structure

- **LaneDetection.py**: Contains the `LaneDetection` class which implements the detection and lane keeping logic.
- **test3.mp4**: Example video used for testing the lane detection and keeping algorithm.
- **requirements.txt**: List of required Python libraries.

## Installation

1. Clone the repository:

    ```bash
    git clone https://github.com/Ziad-Abaza/Lane-Detection-and-Keeping-Algorithm.git
    ```

2. Install the required dependencies:

    ```bash
    pip install -r requirements.txt
    ```

## Usage

1. Ensure you have a video file (`test3.mp4` or any other video) in the appropriate directory.

2. Run the `LaneDetection.py` script:

    ```bash
    python LaneDetection.py
    ```

3. The program will display a real-time window showing the detected lane lines, heading line, and steering direction. Press the **spacebar** to exit the video.

## Functions

- **detect_edges(frame)**: Converts the frame to grayscale, applies Gaussian blur, and detects edges using the Canny edge detector.
- **region_of_interest(edges)**: Crops the image to the region that contains the road and lane markings.
- **detect_line_segments(cropped_edges)**: Detects lane line segments using the Hough Transform.
- **average_slope_intercept(frame, line_segments)**: Averages the slopes and intercepts of detected lines to compute the final lane lines.
- **make_points(frame, line)**: Converts the slope and intercept of a line to points that can be drawn on the image.
- **display_lines(frame, lines)**: Draws the detected lane lines on the frame.
- **display_heading_line(frame, steering_angle)**: Draws the heading line indicating the steering direction.
- **get_steering_angle(frame, lane_lines)**: Computes the steering angle based on the position of detected lane lines.
- **run()**: Main function to continuously process the video frames, detect lane lines, and calculate steering angles.

## Requirements

- Python 3.x
- Libraries:
  - OpenCV (`cv2`)
  - NumPy
  - Math

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
