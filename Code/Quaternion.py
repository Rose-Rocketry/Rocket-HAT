class Quaternion:
    w = 0
    x=0
    y=0
    z=0
    def __init__(self, w, x, y, z):
        self.w = w
        self.x = x
        self.y = y
        self.z = z

    def qconj(self):
        nx = -self.x
        ny = -self.y
        nz = -self.z
        return Quaternion(self.w, nx, ny, nz)

    def qmult(self, b):
        nw = self.w*b.w - self.x*b.x - self.y*b.y - self.z*b.z
        nx = self.w*b.x + self.x*b.w + self.y*b.z - self.z*b.y
        ny = self.w*b.y + self.y*b.w + self.z*b.x - self.x*b.z
        nz = self.w*b.z + self.z*b.w + self.x*b.y - self.y*b.z
        return Quaternion(nw, nx, ny, nz)

    def norm(self):
        mag = (self.x**2 + self.y**2 + self.z**2 + self.w**2) ** 0.5
        self.w = self.w / mag
        self.x = self.x / mag
        self.y = self.y / mag
        self.z = self.z / mag

    def __mul__(self, other):
        return self.qmult(other)

    def __invert__(self):
        return self.qconj()

    def __str__(self):
        result = str(self.w) + " + " + str(self.x) + "i + " + str(self.y) + "j + " + str(self.z) + "k"
        return result
