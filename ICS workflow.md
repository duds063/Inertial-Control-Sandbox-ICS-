ICS workflow

PID_number_Kd = input(int("enter your desired PID constant number Kd: "))
PID_number_Kp = input(int("enter your desired PID constant number Kp: "))
PID_number_Ki = input(int("enter your desired PID constant number Ki: "))
Filter = input(int("Please select the best filters for you    
1- Kalman Filter
2- Gaussian Filter
3- Wiener Filter
4- Adaptative filter
5- Kalman + Gauss
6- Adaptative + Wiener"))
If filter = 1
    apply Kalman filter function
Elif filter = 2
    apply Gaussian filter function
Elif filter = 3
    apply Wiener filter function
Elif filter = 4
    Adaptative_filter = input(int("Please select the type of adaptative filter
    1- Least Mean Squares (LMS)
    2- Recursive Least Squares (RLS)"))
    If Adaptative_filter = 1
        apply LMS filter function
    Elif Adaptative_filter = 2
        apply RLS filter function
    apply Adaptative filter function
Elif filter = 5
    apply Kalman + Gauss filter function
Elif filter = 6
    apply Adaptative + Wiener filter function

