# Keeps the last `maxlen` things added - used to spot a color pattern
# (like GREEN then WHITE) on the mat.

class CircularBuffer:
    def __init__(self, maxlen):
        self.maxlen = maxlen
        self.data = []

    def append(self, item):
        self.data.append(item)
        if len(self.data) > self.maxlen:
            self.data.pop(0)

    # 1. Allows direct comparison using ==
    def __eq__(self, other):
        if isinstance(other, list):
            return self.data == other
        if isinstance(other, CircularBuffer):
            return self.data == other.data
        return False

    # 2. Allows len(buffer)
    def __len__(self):
        return len(self.data)

    # 3. Allows item indexing (buffer[0]) and "item in buffer" checks
    def __getitem__(self, index):
        return self.data[index]
    def __repr__(self):
        return repr(self.data)
