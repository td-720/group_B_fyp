from rosbags.rosbag2 import Reader

bag_path = '/home/theo/ros2_ws/gui/rosbag2_2026_07_10-11_36_36'

with Reader(bag_path) as reader:

    duration = reader.duration / 1e9
    print("Duration:", duration, "seconds")

    for connection in reader.connections:
        print("\nTopic:", connection.topic)

        count = 0

        for _, _, _ in reader.messages([connection]):
            count += 1

        frequency = count / duration

        print("Messages:", count)
        print("Frequency:", round(frequency, 2), "Hz")