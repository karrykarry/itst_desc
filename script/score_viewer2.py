
#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Float64

price = [100, 250, 380, 500, 700]
number = [1, 2, 3, 4, 5]

class Score_view():
    def __init__(self):
        self.plt.title("price/number")
        self.plt.xlabel("price")
        self.plt.ylabel("price")

    def viewer(self, otsu_msg):
        self.plt.plot(price, number)
        
        self.plt.show()
        

    def callback(self, msg):
        self.viewer(msg)


    def listen(self):
        print("listen")
        rospy.Subscriber("/message", Float64, callback)
        print("subscribed")
        rospy.spin()


    def main(self):
        rospy.init_node('score_viewer', anonymous=True)
        self.listen()


if __name__ == '__main__':
    sc = Score_view()
    
    sc.main()
