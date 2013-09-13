import numpy as np
import matplotlib.pyplot as plt
import matplotlib.mlab as ml

class CoordinateCorrector:
    def __init__(self,enable = True, filename = "error points orig"):
        self.enable = enable
        if enable:
            self.x_data, self.y_data, self.z_data = getInterpolators(filename)
    
    def correctCoords(self,pixel_x, pixel_y,x,y,z):
        if not self.enable:
            return x,y,z
        new_x = x - self.x_data.getValue(pixel_x,pixel_y)
        new_y = y - self.y_data.getValue(pixel_x,pixel_y)
        new_z = z - self.z_data.getValue(pixel_x,pixel_y)
        return new_x, new_y, new_z
    def getErrors(self,x,y,z):
        if not self.enable:
            return 0,0,0
        return self.x_data.getValue(x,y), self.y_data.getValue(x,y), self.z_data.getValue(x,y)

class Interpolator:
    def __init__(self,x,y,z,xmax = None, ymax = None,nx = 200, ny = 200):
        if xmax == None:
            xmax = max(x) * 1.1
        if ymax == None:    
            ymax = max(y) * 1.1
        self.xmax = xmax
        self.ymax = ymax
        self.xi = np.linspace(0, xmax, nx)
        self.yi = np.linspace(0, ymax, ny)
        self.zi = ml.griddata(x, y, z, self.xi, self.yi)
        self.nx = nx
        self.ny = ny
        self.x = x
        self.y = y
        
    def plot(self):
        plt.contour(self.xi, self.yi, self.zi, 15, linewidths = 0.5, colors = 'k')
        plt.pcolormesh(self.xi, self.yi, self.zi, cmap = plt.get_cmap('rainbow'))
        plt.colorbar() 
        plt.scatter(self.x, self.y, marker = 'o', c = 'b', s = 5, zorder = 10)
        plt.xlim(0, self.xmax)
        plt.ylim(0, self.ymax)
        plt.show()
        
    def getValue(self,x,y):
        x_index = round(x/self.xmax * self.nx)
        y_index = round(y/self.ymax * self.ny)
        print "x_index", x_index, "y_index", y_index
        if self.zi[y_index][x_index]:
            return self.zi[y_index][x_index]
        #otherwise try some values around it, as a poor mans guessing techinque
        #there would be better ways of doing this, but this shouldn't happen under most circumstances
        new_x = round(x_index * 0.8 + self.xmax * 0.1)
        new_y = round(y_index * 0.8 + self.ymax * 0.1)
        
        return 0
        
        print "making a guess...."
        if self.zi[new_y][x_index]:
            return self.zi[new_y][x_index]
        
        if self.zi[y_index][new_x]:
            return self.zi[y_index][new_x]
        
        if self.zi[new_y][new_x]:
            return self.zi[new_y][new_x]
        #finally return 0 if that doesn't work (ideally this would never happen)
        return 0
        
def getInterpolators(filename):
    x = []
    y = []
    x_error = []
    y_error = []
    z_error = []
    with open(filename) as f:
        for chunk in f:#may read one line, or many (depending on weirdness with \r or \n or \r\n
            
            if "\r\n" in chunk:
                terminator = "\r\n"
            elif "\r" in chunk:
                terminator = "\r"
            else:
                terminator = "\n"
            chunk = chunk.rstrip()
            lines = chunk.split(terminator)
            for line in lines:
                #print "line = ", line
                vals = []
                for str_val in line.split(","):
                    #print str_val
                    vals.append(float(str_val))
                x_error.append(vals[0])
                y_error.append(vals[1])
                z_error.append(vals[2])
                x.append(vals[3])
                y.append(vals[4])
            
    x_data = Interpolator(x,y,x_error)
    y_data = Interpolator(x,y,y_error)
    z_data = Interpolator(x,y,z_error)
    return x_data, y_data, z_data


if __name__ == "__main__":
    c = CoordinateCorrector(True,"error points")
    c.x_data.plot()
    c.y_data.plot()
    c.z_data.plot()
