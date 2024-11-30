import numpy as np 
import matplotlib.pyplot as plt 
import networkx as nx
import yaml
from PIL import Image 


def load_yaml(yaml_path):
    
    with open(yaml_path, 'r') as file:
        return yaml.safe_load(file)
    
def pgm_to_occupancy(pgm_path):

    img = Image.open(pgm_path)  

    grid = np.array(img)

    occupancy_grid = grid / grid.max()

    return occupancy_grid

def visualization(occ_grid):
    plt.imshow(occ_grid, cmap='YlGnBu')
    plt.colorbar(label='Occupancy Probability')
    plt.title('Occupancy Grid')
    plt.show()    




def main():

    occupancy_grid = pgm_to_occupancy('/home/autodrive_devkit/src/car_control/car_control/maps/iros_2024/iros_map_compete2024.pgm')

    visualization(occupancy_grid)




if __name__ ==  '__main__':

    main()
