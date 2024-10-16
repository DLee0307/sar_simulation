import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.animation as animation
from matplotlib.patches import Rectangle
import os
import cv2

## PLOT BRIGHTNESS PATTERN FROM 2.4.1 HORIZONTAL MOTION
I_0 = 255   # Brightness value (0-255)

## CAMERA PARAMETERS
FoV = 82.22 # Field of View [deg]
FoV_rad = np.radians(FoV) # Field of View [deg]

WIDTH_PIXELS = 32
HEIGHT_PIXELS = 32
w = 3.6e-6              # Pixel width [m]
f = 0.066e-3             # Focal Length [m]
O_up = WIDTH_PIXELS/2   # Pixel X_offset [pixels]
O_vp = HEIGHT_PIXELS/2  # Pixel Y_offset [pixels]


class Optical_Flow():

    def __init__(self,L,FPS):
        """_summary_

        Args:
            L (float):  [m/s]
            FPS (uint): Frames per second
        """              

        self.L = L*2 # Double the Feature width for sine period
        self.FPS = FPS

    def Generate_Camera_Image(self,D_cam=1,X_cam=0):

        U_p,V_p = np.meshgrid(np.arange(0,WIDTH_PIXELS,1),np.arange(0,HEIGHT_PIXELS,1))

        ## CONVERT PIXEL INDEX TO METERS
        u =  (U_p - O_up)*w + w/2 
        v =  (U_p - O_up)*w + w/2

        ## RETURN AVERAGE BRIGHTNESS VALUE OVER PIXEL
        I = I_0/2 * (f*self.L/(np.pi*D_cam*w) * (np.sin(np.pi*D_cam*w/(self.L*f))*np.sin(2*np.pi/self.L * (D_cam*u/f + X_cam))) + 1)

        return I

    def Generate_Pattern(self,Surf_width=2,Surf_Height=2,pixel_density=100,save_img=False,X_cam=0.0,D_cam=0.5):
        """Save show pattern and save image to file. As well, display camera boundaries for validation

        Args:
            Surf_width (int, optional): Width of pattern to preserve pixel density. Defaults to 2.
            Surf_Height (int, optional): Heigt of pattern to preserve pixel density. Defaults to 2.
            pixel_density (int, optional): Number of pixels per meter. Defaults to 100.
            save_img (bool, optional): Save image to file. Defaults to False.
            X_cam (float, optional): X location of camera. Defaults to 0.5.
            D_cam (float, optional): Distance of camera from pattern surface. Defaults to 0.5.
        """        

        ## GENERATE PATTERN BOUNDS
        x = np.linspace(-0.5*Surf_width, 0.5*Surf_width, pixel_density*Surf_width)
        y = np.linspace(-0.5*Surf_Height,0.5*Surf_Height,pixel_density*Surf_Height)
        X,Y = np.meshgrid(x,y)

        ## GENERATE PATTERN
        I = I_0/2*(np.sin(2*np.pi/self.L*X) + 1)
        # I = np.where(I < 128,0,255)

        ## GENERATE CAMERA BOUNDS
        Img_Width = 2*np.tan(FoV_rad/2)*D_cam
        Img_Height = 2*np.tan(FoV_rad/2)*D_cam

        
        ## CREATE FIGURE OF PATTERN
        fig = plt.figure()
        ax = fig.add_subplot(111)

        ## PLOT PATTERN AND CAMERA BOUNDARY
        ax.imshow(I, interpolation='none', 
            vmin=0, vmax=255, cmap=cm.gray,
            origin='upper',
            extent=[x.min(),x.max(),y.min(),y.max()]
            )

        ax.add_patch(
            Rectangle(
                (X_cam-Img_Width/2,-Img_Height/2),
                Img_Width,
                Img_Height,
                lw=1,fill=False,color="tab:blue")
            )

        ## SHOW PLOT
        plt.show()

        ## SAVE PATTERN TO FILE
        if save_img == True:
            plt.imsave(
                f'/home/dlee/Pictures/project/Strip_Pattern_W_{Surf_width}-H_{Surf_Height}.png', 
                I, 
                vmin=0, vmax=255, cmap=cm.gray,
                origin='upper',
            )


    def Optical_Flow_Traj(self,D_0,Vz,X_0,Vx):

        Tau_List = []
        Tau_est_List = []
        Theta_x_List = []
        Theta_x_est_List = []
        t_List = []

        D_perp_prev = D_0
        X_cam_prev = X_0

        t_max = 1

        ## IMAGE SENSOR PLOT
        fig = plt.figure()
        ax = fig.add_subplot(111)
        im = ax.imshow(self.Generate_Camera_Image(D_cam=D_0,X_cam=X_0), interpolation='none', 
                        vmin=0, vmax=255, cmap=cm.gray,
                        origin='upper',
        )
        ax.set_title("Image Sensor Pattern (Pixels)")
        ax.set_xlabel("u_p [pixels]")
        ax.set_ylabel("v_v [pixels]")
        fig.tight_layout()

        def animate_func(i):

            ## UPDATE CAMERA POSITION
            t = i/self.FPS   # Time [s]

            D_perp = D_0 - Vz*t    # Distance to Ceiling [m]
            X_cam = X_0 + Vx*t

            D_perp_prev = D_0 - Vz*(t-1/self.FPS)
            X_cam = X_0 + Vx*(t-1/self.FPS)


            Tau = D_perp/(Vz+1e-6)
            Theta_x = Vx/D_perp

            ## STOP ANIMATION IF PAST CEILING SURFACE
            if D_perp <= 0.01:
                return
            
            ## CALCULATE OPTICAL FLOW ESTIMATES
            Prev_img = self.Generate_Camera_Image(D_cam=D_perp_prev,X_cam=X_cam_prev)
            Cur_img = self.Generate_Camera_Image(D_cam=D_perp,X_cam=X_cam)
            b = self.OF_Calc_Opt_Sep(Cur_img,Prev_img,1/self.FPS)
            Theta_x_est,Theta_y_est,Theta_z_est = b
            Tau_est = 1/Theta_z_est
            
            print(f"Tau: {Tau:.3f} | Tau_est: {Tau_est:.3f}")
            print(f"Theta_x: {Theta_x:.3f} | Theta_x_est: {Theta_y_est:.3f}\n")


            ## APPEN OPTICAL FLOW ESTIMATES TO LIST FOR PLOTTING
            Tau_List.append(Tau)
            Theta_x_List.append(Theta_x)

            Tau_est_List.append(Tau_est)
            Theta_x_est_List.append(-Theta_y_est)
       
            t_List.append(t)

            ## UPDATE IMAGE
            im.set_array(Cur_img)
            
        anim = animation.FuncAnimation(fig, 
                                    animate_func, 
                                    frames = t_max * self.FPS,
                                    interval = 1000 / self.FPS, # in ms
                                    repeat = False
                                    )

        plt.show()

        ## TAU PLOT
        fig2 = plt.figure(1)
        ax = fig2.add_subplot(111)

        ax.plot(t_List,Tau_est_List,'rx',label="Tau_estimate")
        ax.plot(t_List,Tau_List,label="Tau_actual")
        # ax.set_title(f'Tau Estimation - V: {vel:.2f} | Phi: {phi:.2f} | L: {L:.3f}')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Tau [s]')
        ax.grid()
        ax.legend()

        fig2.tight_layout()

    def OF_Calc_Opt_Sep(self,cur_img,prev_img,delta_t):

        """Calculate optical flow values with seperable convolution and integer optimizations.
        Derivation in (Research_Notes_Book_2.pdf)

        Args:
            cur_img (np.array): Array of current image
            prev_img (np.array): Array of previous image
            delta_t (float): Time between images

        Returns:
            np.array: Array of (Theta_x_est,Theta_y_est,Theta_z_est)
        """        

        ## SEPERATED SOBEL KERNAL (U--DIRECTION)
        Ku_1 = np.array([[-1,0,1]]).reshape(3,1)
        Ku_2 = np.array([[ 1,2,1]]).reshape(1,3)


        ## SEPERATED SOBEL KERNAL (V--DIRECTION)
        Kv_1 = np.array([[ 1,2,1]]).reshape(3,1)
        Kv_2 = np.array([[-1,0,1]]).reshape(1,3)


        ## PRE-ALLOCATE INTENSITY GRADIENTS
        G_up = np.zeros((HEIGHT_PIXELS,WIDTH_PIXELS))
        G_vp = np.zeros((HEIGHT_PIXELS,WIDTH_PIXELS))
        G_rp = np.zeros((HEIGHT_PIXELS,WIDTH_PIXELS))
        G_tp = np.zeros((HEIGHT_PIXELS,WIDTH_PIXELS))


        ## CALCULATE IMAGE GRADIENTS
        for v_p in range(1, HEIGHT_PIXELS-1): 
            for u_p in range(1, WIDTH_PIXELS-1):
                G_up[v_p,u_p] = (Ku_2.dot((cur_img[v_p-1:v_p+2,u_p-1:u_p+2].dot(Ku_1)))).item()
                G_vp[v_p,u_p] = (Kv_2.dot((cur_img[v_p-1:v_p+2,u_p-1:u_p+2].dot(Kv_1)))).item()
                G_rp[v_p,u_p] = (2*(u_p - O_up) + 1)*G_up[v_p,u_p] + (2*(v_p - O_vp) + 1)*G_vp[v_p,u_p]
                G_tp[v_p,u_p] = cur_img[v_p,u_p] - prev_img[v_p,u_p]

        np.savetxt('/home/dlee/Pictures/project/G_up_matrix.csv', G_up, delimiter=',', fmt='%.6f')
        np.savetxt('/home/dlee/Pictures/project/G_vp_matrix.csv', G_vp, delimiter=',', fmt='%.6f')
        np.savetxt('/home/dlee/Pictures/project/G_rp_matrix.csv', G_rp, delimiter=',', fmt='%.6f')
        np.savetxt('/home/dlee/Pictures/project/G_tp_matrix.csv', G_tp, delimiter=',', fmt='%.6f')

        ## SOLVE LEAST SQUARES PROBLEM
        X = np.array([
            [f*np.sum(G_vp*G_vp), -f*np.sum(G_up*G_vp), -w/2*np.sum(G_rp*G_vp)],
            [f*np.sum(G_vp*G_up), -f*np.sum(G_up*G_up), -w/2*np.sum(G_rp*G_up)],
            [f*np.sum(G_vp*G_rp), -f*np.sum(G_up*G_rp), -w/2*np.sum(G_rp*G_rp)]
        ])

        y = np.array([
            [np.sum(G_tp*G_vp)],
            [np.sum(G_tp*G_up)],
            [np.sum(G_tp*G_rp)]
        ])*(8*w/delta_t)

        np.savetxt('/home/dlee/Pictures/project/X_matrix.csv', X, delimiter=',', fmt='%.6f')
        np.savetxt('/home/dlee/Pictures/project/y_vector.csv', y, delimiter=',', fmt='%.6f')

        ## SOLVE b VIA PSEUDO-INVERSE
        b = np.linalg.pinv(X)@y
        b = b.flatten()

        return b

if __name__ == '__main__':

    OF = Optical_Flow(L=0.25,FPS=60)
    #OF.Generate_Pattern(D_cam=1.0,Surf_width=4,Surf_Height=4,save_img=True)
    Cur_img_1 = OF.Generate_Camera_Image(D_cam=0.9, X_cam=0)
    plt.imsave('/home/dlee/Pictures/project/Generated_Cur_img_1.png', Cur_img_1, cmap=cm.gray, vmin=0, vmax=255)
    np.savetxt('/home/dlee/Pictures/project/Generated_Cur_img_1.csv', Cur_img_1, delimiter=',', fmt='%.2f')

    Prev_img_1 = OF.Generate_Camera_Image(D_cam=1, X_cam=0)
    plt.imsave('/home/dlee/Pictures/project/Generated_Prev_img_1.png', Prev_img_1, cmap=cm.gray, vmin=0, vmax=255)
    np.savetxt('/home/dlee/Pictures/project/Generated_Prev_img_1.csv', Prev_img_1, delimiter=',', fmt='%.2f')

    cur_img_path = '/home/dlee/Pictures/project/Generated_Cur_img_1.png'
    cur_img_cv = cv2.imread(cur_img_path, cv2.IMREAD_GRAYSCALE)
    if cur_img_cv is not None:
        cur_img_cv = cur_img_cv.astype(np.float64) 
        np.savetxt('/home/dlee/Pictures/project/Loaded_Cur_img_1.csv', cur_img_cv, delimiter=',', fmt='%.2f')
        print("Current image loaded and saved as CSV successfully.")
    else:
        print(f"Error loading current image: {cur_img_path}")

    # Load the previous image
    prev_img_path = '/home/dlee/Pictures/project/Generated_Prev_img_1.png'
    prev_img_cv = cv2.imread(prev_img_path, cv2.IMREAD_GRAYSCALE)
    if prev_img_cv is not None:
        prev_img_cv = prev_img_cv.astype(np.float64)
        np.savetxt('/home/dlee/Pictures/project/Loaded_Prev_img_1.csv', prev_img_cv, delimiter=',', fmt='%.2f')
        print("Previous image loaded and saved as CSV successfully.")
    else:
        print(f"Error loading previous image: {prev_img_path}")

    
    #np.savetxt('/home/dlee/Pictures/project/X_matrix.csv', X, delimiter=',', fmt='%.6f')
    #np.savetxt('/home/dlee/Pictures/project/y_vector.csv', y, delimiter=',', fmt='%.6f')

    b = OF.OF_Calc_Opt_Sep(cur_img_cv,prev_img_cv,1/60)
    Theta_x_est,Theta_y_est,Theta_z_est = b
    Tau_est = 1/Theta_z_est
    print()
    


    OF.Optical_Flow_Traj(X_0=0,Vx=0,D_0=1.0,Vz=1.0)

    #anim = OF.Optical_Flow_Traj(X_0=0,Vx=0.0,D_0=0.5,Vz=0.1)

    #plt.show()