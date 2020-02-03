import pandas as pd
import numpy as np



def computeRMSE(trueVector, EstimateVector):

    sum = 0
    for i in range(len(trueVector)):
        sum += (trueVector[i] - EstimateVector[i]) ** 2
    sum *= 1.0 / len(trueVector)
    return np.sqrt(sum)


def computeRadarJacobian(Xvector):
    Px , Py , Vx , Vy = Xvector[0] , Xvector[1] , Xvector[2] , Xvector[3]
    jacobian = np.array(
        [
            [Px / np.sqrt(Px ** 2 + Py ** 2), Py / np.sqrt(Px ** 2 + Py ** 2), 0, 0],
            [-Py / (Px ** 2 + Py ** 2), Px / (Px ** 2 + Py ** 2), 0, 0],
            [(Py * (Vx * Py - Vy * Px)) / np.power(Px ** 2 + Py ** 2, 1.5),
                                (Px * (Vy * Px - Vx * Py) / np.power(Px ** 2 + Py ** 2, 1.5)),
                                 Px / np.sqrt(Px ** 2 + Py ** 2),
                                 Py / np.sqrt(Px ** 2 + Py ** 2)]
        ])
    return jacobian


def computeCovMatrix(deltaT, sigma_aX, sigma_aY):
    x = np.array([[((deltaT ** 4 / 4.) * (sigma_aX ** 2)), 0, ((deltaT ** 3 / 2.) * (sigma_aX ** 2)), 0],
                  [0, ((deltaT ** 4 / 4.) * (sigma_aY ** 2)), 0, ((deltaT ** 3 / 2.) * (sigma_aY ** 2))],
                  [((deltaT ** 3 / 2.) * (sigma_aX ** 2)), 0, ((deltaT ** 2) * (sigma_aX ** 2)), 0],
                  [0, ((deltaT ** 3 / 2.) * (sigma_aY ** 2)), 0, ((deltaT ** 2) * (sigma_aY ** 2))]])
    return x


def computeFmatrix(deltaT):
    dt = deltaT
    F = np.array([[1., 0., dt, 0.],
                  [0., 1.0, 0., dt],
                  [0., 0., 1., 0.],
                  [0., 0., 0., 1.]])  # next state function: generalize the 2d version to 4d
    return F



def computeZradar(Xvector):
    Px , Py , Vx , Vy = Xvector[0] , Xvector[1] , Xvector[2] , Xvector[3]
    Hradar = np.array([np.sqrt(Px**2 + Py**2),
                       np.arctan(Py/Px),
                       (Px*Vx+Py*Vy) / np.sqrt(Px**2+Py**2)])
    return Hradar

def main():
    # we print a heading and make it bigger using HTML formatting
    my_cols = ["A", "B", "C", "D", "E", "f", "g", "h", "i", "j", "k"]
    data = pd.read_csv("./data.txt", names=my_cols, delim_whitespace=True, header=None)
    #print(data.head())
    for i in range(10):
        measur = data.iloc[i, :].values
    
    
    deltaT = 0.1
    useRadar = True

    I = np.array([[1., 0., 0., 0.], [0., 1., 0., 0.], [0., 0., 1., 0.], [0., 0., 0., 1.]])  # Identity matrix
    P = np.array([[0., 0., 0., 0.], [0., 0., 0., 0.], [0., 0., 1000.,1000],[0., 0., 1000., 1000.]])
    xEstimate = []
    xTrue = []
    H_Lidar = np.array([[1., 0., 0., 0.], [0., 1., 0., 0.]])

    R_lidar = np.array([[0.0225, 0.0], [0.0, 0.0225]])
    R_radar = np.array([[0.9, 0, 0], [0.0, 0.0009, 0], [0, 0, 0.09]])

    F_matrix = computeFmatrix(deltaT)
    X_true_current = np.array([3.122427e-01, 5.803398e-01, 0, 0]) # The initial state
    firstMeasurment = data.iloc[0, :].values
    timeStamp = firstMeasurment[3]

    X_state_current = np.array([3.122427e-01, 5.803398e-01, 0, 0]) # The initial state

    for i in range(1, len(data)):
        currentMeas = data.iloc[i, :].values
        # compute the current dela t
        if (currentMeas[0] == 'L'):
            deltaT = (currentMeas[3] - timeStamp) / 1000000
            timeStamp = currentMeas[3]

             # perfrom predict
            X_state_current = F_matrix @ X_state_current  # TODO add noise u
            P = F_matrix @ P @ F_matrix.transpose()

            # pefrom measurment update
            z = np.array([currentMeas[1], currentMeas[2]])  
            # array of [x_measure,y_measure]
            y = z.transpose() - (H_Lidar @ X_state_current)
            R = R_lidar  
            # Measurment noise matrix
            S = H_Lidar @ P @ H_Lidar.transpose()  + R
            K = P @ H_Lidar.transpose() @ np.linalg.inv(S)
            X_state_current = X_state_current + (K @ y)
            Q=computeCovMatrix(deltaT,9,9)
            P = (I - (K @ H_Lidar)) @ P 
            #with noise 
            
            #P = (I - (K @ H_Lidar)) @ P + Q
            xEstimate.append(X_state_current)
            X_true_current = np.array([currentMeas[4], currentMeas[5], currentMeas[6], currentMeas[7]])
            xTrue.append(X_true_current)


        if (currentMeas[0] == 'R' and useRadar):

            # perfrom predict
            deltaT = (currentMeas[4] - timeStamp) / 1000000
            timeStamp = currentMeas[4]
            X_state_current = F_matrix @ X_state_current
            P = F_matrix @ P @ F_matrix.transpose()

            # pefrom measurment update
            # array of [rho_measured, phi_measured, rhodot_measured]
            jacobian = computeRadarJacobian(X_state_current)
            z = np.array([currentMeas[1], currentMeas[2] , currentMeas[3]])  # array of [rho_measured, phi_measured, rhodot_measured]
            z = computeZradar(X_state_current)   # convert from space state to sensor state
            R = R_radar                          
            H = jacobian
            y = z.transpose() - (H @ X_state_current)
            S = H @ P @ H.transpose() + R
            K = P @ H.transpose() @ np.linalg.inv(S)
            X_state_current = X_state_current + (K @ y)
            P = (I - (K @ H)) @ P


            
            xEstimate.append(X_state_current)
            X_true_current = np.array([currentMeas[5], currentMeas[6], currentMeas[7], currentMeas[8]])
            xTrue.append(X_true_current)


    rmse = computeRMSE(xEstimate, xTrue)
    print("The RMSE is:",rmse)


if __name__ == '__main__':
    main()
