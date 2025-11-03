
#Jane D'Souza
#Final Project (Deliverable 2)
#Student number: 400366436
#April 2024

import open3d as o3d
import numpy as np


 
def create_lines(vertices, measurements_per_layer):

    lines = []#intialize empty list to store lien connections between points 


    num_vertices = len(vertices)#calcute total num of vertices by dividng total vertics=es by num of measurements by layer

    layers = num_vertices // measurements_per_layer

    #Connect points within the same layers

    for layer in range(layers):

        for i in range(measurements_per_layer):

            idx = layer * measurements_per_layer + i#caclute current index in the point 


#calculate next index within the same layer or wrap around to the start of the layer 
            next_idx = idx + 1 if (i + 1) < measurements_per_layer else layer * measurements_per_layer

            lines.append([idx, next_idx]) #append the line connection to the list

    # Connect corresponding points in consecutive layers

    for i in range(measurements_per_layer):

        for layer in range(layers - 1):

            idx = layer * measurements_per_layer + i

            upper_idx = idx + measurements_per_layer  #index directly above curr index in the next layer 


            lines.append([idx, upper_idx])

    return lines
 
if __name__ == "__main__":

    # Read test data from the files created
    #print message idnciating the script has started and will read PCD (point cloud data)

    print("Read in the prism point cloud data (pcd)")

    pcd = o3d.io.read_point_cloud("output.xyz", format="xyz") #load PC file located at output.xyz 


    # print statment to let us know that the PCD will be visualized 

    print("Let's visualize the PCD: (spawns separate interactive window)")

    o3d.visualization.draw_geometries([pcd])

    #convert point cloud files to NumPy array for easier manipulation (extract)

    vertices = np.asarray(pcd.points)

    measurements_per_layer = 32  # measurement defintion for layer (how lines are connected)

    lines = create_lines(vertices, measurements_per_layer) #make line connections based on vertices and defined structure 


    #Create a lineSet object to maange and see connections (visulaization)

    line_set = o3d.geometry.LineSet(

        points=o3d.utility.Vector3dVector(vertices),

        lines=o3d.utility.Vector2iVector(lines)

    )

    #print statement indicating that line set will be visualzied

    print("Visualizing the line set:")

    o3d.visualization.draw_geometries([line_set]) #see the line conneections in open3D

