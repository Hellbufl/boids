import numpy as np

def contained(item_pos, item_size, area_pos, area_size):
    return (item_pos > area_pos).all() and (item_pos + item_size < area_pos + area_size).all()

def overlap(item_pos, item_size, area_pos, area_size):
    return (item_pos < area_pos + area_size).all() and (item_pos + item_size > area_pos).all()

class QuadTree:

    def __init__(self, layer, size, pos = np.zeros(2), parent = None):

        self.position = pos
        self.size = size

        self.items = np.array([])
        self.item_positions = np.array([[]])
        self.item_sizes = np.array([[]])

        self.children = np.array([None, None, None, None])
        self.layer = layer
    
    def reset(self):
        self.__init__(self.layer, self.size)

    def get_items(self):
        return self.items
    
    def get_all_items(self):
        items = self.items
        for i in range(4):
            if self.children[i]:
                items = np.append(items, self.children[i].get_all_items())
        return items

    def get_position(self):
        return self.position
    
    def get_size(self):
        return self.size
    
    def set_item(self, item, item_position, item_size):
        if self.items.size == 0:
            self.items = np.array([item])
            self.item_positions = np.array([item_position])
            self.item_sizes = np.array([item_size])
        else:
            self.items = np.append(self.items, [item], 0)
            self.item_positions = np.append(self.item_positions, [item_position], 0)
            self.item_sizes = np.append(self.item_sizes, [item_size], 0)
    
    def insert_array(self, items, item_positions, item_sizes):
        for i in range(items.size):
            self.insert(items[i], item_positions[i], item_sizes[i])
    
    def insert(self, item, item_position, item_size):
        if self.layer == 0:
            self.set_item(item, item_position, item_size)
            return [(self.position, self.size)]
        
        local_position = item_position - self.position
        
        child_size = self.size / 2
        child_pos = self.position + (child_size) * (local_position > child_size)
        child_index = np.sum((local_position > child_size) * [1, 2])

        if (child_pos + child_size - item_size < item_position).any():
            self.set_item(item, item_position, item_size)
            return [(self.position, self.size)]

        if type(self.children[child_index]) == type(None):
            self.children[child_index] = QuadTree(self.layer - 1, self.size / 2, child_pos, self)

        return [(self.position, self.size)] + self.children[child_index].insert(item, item_position, item_size)
    
    def search(self, area_position, area_size):
        found = np.array([], dtype=int)

        if self.items.size > 0:
            mask = np.zeros(self.items.shape[0], dtype=bool)
            for i in range(self.items.shape[0]):
                if overlap(self.item_positions[i], self.item_sizes[i], area_position, area_size):
                    mask[i] = True

            found = self.items[mask]

        if self.layer > 0:

            for i in range(4):

                if type(self.children[i]) != type(None):
                    if contained(self.children[i].get_position(), self.children[i].get_size(), area_position, area_size):
                        found = np.append(found, self.children[i].get_all_items())

                    elif overlap(self.children[i].get_position(), self.children[i].get_size(), area_position, area_size):
                        found = np.append(found, self.children[i].search(area_position, area_size))

        return np.array(found, dtype=int)