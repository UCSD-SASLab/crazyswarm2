import os
import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

import matplotlib.pyplot as plt
import numpy as np

def find_db3_files(directory):
    db3_files = []
    db3_dirs = []
    for root, dirs, files in os.walk(directory):
        for dir in dirs:
            db3_dirs.append(dir)
        for file in files:
            if (file.endswith(".db3")):
                full_path = os.path.join(root, file)
                db3_files.append(full_path)
    return db3_files, db3_dirs

class BagFileParser():
    def __init__(self, bag_file):
        self.conn = sqlite3.connect(bag_file)
        self.cursor = self.conn.cursor()

        ## create a message type map
        topics_data = self.cursor.execute("SELECT id, name, type FROM topics").fetchall()
        self.topic_type = {name_of:type_of for id_of,name_of,type_of in topics_data}
        self.topic_id = {name_of:id_of for id_of,name_of,type_of in topics_data}
        self.topic_msg_message = {name_of:get_message(type_of) for id_of,name_of,type_of in topics_data}

    def __del__(self):
        self.conn.close()

    # Return [(timestamp0, message0), (timestamp1, message1), ...]
    def get_messages(self, topic_name):

        topic_id = self.topic_id[topic_name]
        # Get from the db
        rows = self.cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = {}".format(topic_id)).fetchall()
        # Deserialise all and timestamp them
        return [ (timestamp,deserialize_message(data, self.topic_msg_message[topic_name])) for timestamp,data in rows]



if __name__ == "__main__":
        wd = os.getcwd() + '/raw_data/'
        db3_file_paths, db3_dirs = find_db3_files(wd)
        print(db3_dirs)
        for c, path in enumerate(db3_file_paths):
            parser = BagFileParser(db3_file_paths[c])

            traj = parser.get_messages("/cf231/pose")
            print(len(traj))
            
            pose = np.empty((0, 7))
            indices = []

            for i in range(0, len(traj)):
                x = traj[i][1].pose.position.x
                y = traj[i][1].pose.position.y
                z = traj[i][1].pose.position.z

                x_theta = traj[i][1].pose.orientation.x
                y_theta = traj[i][1].pose.orientation.y
                z_theta = traj[i][1].pose.orientation.z
                w = traj[i][1].pose.orientation.w

                curr_pos = np.array([[x, y, z, x_theta, y_theta, z_theta, w]])
                pose = np.append(pose, curr_pos, axis=0)
                indices.append(i) 
                np.save(f"parsed_data/{db3_dirs[c][-18:]}", pose)

            fig = plt.figure(figsize=(10, 6))
            ax = fig.add_subplot(111, projection='3d')

            ax.scatter(pose[:, 0], pose[:, 1], pose[:, 2], c=indices, cmap='viridis', label='Trajectory Points')

            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            ax.set_title('3D Trajectory Plot')

            cbar = plt.colorbar(ax.scatter(pose[:, 0], pose[:, 1], pose[:, 2], c=indices, cmap='viridis'))
            cbar.set_label('Index (i)')

            plt.show()