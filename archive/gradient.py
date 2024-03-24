import numpy as np
from scipy import optimize
import scipy.optimize as opt
from scipy.optimize import root

###Probleme 1====================================================================================

# Fonction J(x1, x2)
def J(x1, x2):
    """une banane de Rosenbrock modifiée"""
    return (x1 - 1)**2 + 2 * (x1**2 - x2)**2

# Gradient de J(x1, x2)
def gradient_J(x1, x2):
    """calculé à la main"""
    df_dx1 = 2 * (x1 - 1) + 2*2*2*x1* (x1**2 - x2)
    df_dx2 = -2*2 * (x1**2 - x2)
    return np.array([df_dx1, df_dx2])

# Algorithme du gradient à pas fixe
def gradient_descent(obj_func, gradient_func, initial_point, alpha, epsilon, max_iter):
    """ obj_func: fonction qui calcule J,
        gradient_func: fonction qui calcule son gradient,
        initial_point: le point de départ de l'algorithme,
        alpha: la valeur du pas,
        epsilon: la précision souhaitée,
        max_iter: le nombre maximal d'itérations autorisées.
        elle renvoie la suite de points (Xn) ainsi qu'un indicateur de convergence."""
    x_n = np.array(initial_point)
    dx = 1
    n = 0
    path = [x_n]

    while dx > epsilon and n < max_iter:
        gradient = gradient_func(*x_n)
        x_new = x_n - alpha * gradient
        dx = np.linalg.norm(x_new - x_n)
        x_n = x_new
        n += 1
        path.append(x_n)
    converged = (dx <= epsilon)
    return path, converged

# On modifie l'Algorithme du gradient à pas fixe pour obtenir le nombre d'n_iterations
def gradient_descent_n(obj_func, gradient_func, initial_point, alpha, epsilon, max_iter):
    """ obj_func: fonction qui calcule J,
        gradient_func: fonction qui calcule son gradient,
        initial_point: le point de départ de l'algorithme,
        alpha: la valeur du pas,
        epsilon: la précision souhaitée,
        max_iter: le nombre maximal d'itérations autorisées.
        elle renvoie la suite de points (Xn) ainsi qu'un indicateur de convergence."""
    x_n = np.array(initial_point)
    dx = 1
    n = 0
    path = [x_n]

    while dx > epsilon and n < max_iter:
        gradient = gradient_func(*x_n)
        x_new = x_n - alpha * gradient
        dx = np.linalg.norm(x_new - x_n)
        x_n = x_new
        n += 1
        path.append(x_n)
    converged = (dx <= epsilon)
    return path, converged,n

###Probleme2====================================================================================
def base_fun(x):
    """x = [u1, u2, v1, v2, l1, l2]"""
    return [
        2*(x[0]-x[1]) + x[4],
        -2*(x[0]-x[1]) - 2*x[5]*(x[1]-3),
        2*(x[2]-x[3]) + x[4]*(x[2]-2),
        -2*(x[2]-x[3]) + x[5],
        x[0] + 0.5 * (x[2]-2)**2-1,
        x[3] - (x[1]-3)**2 -2
        ]



###Probleme3====================================================================================


# Fonction J(X), X = [x1, x2, lambda]
def J3(X):
    x = X[0]
    y = X[1]
    return (x- 1)**2 + 2 * (x**2 - y)**2

# Gradient de J(X), X = [x1, x2, lambda]
def gradient_J3(X):
    x = X[0]
    y = X[1]
    dJxn = 2 * (x - 1) + 2*2*2*x* (x**2 - y)
    dJyn = -2*2 * (x**2- y)
    return np.array([dJxn, dJyn])

def systeme(X_var, param):
    """
    X = [x, y, lambda]
    param = [xn, yn, dJxn,dJyn]
    """
    x = X_var[0]
    y = X_var[1]
    l = X_var[2]
    xn = param[0]
    yn = param[1]
    dJxn = param[2]
    dJyn = param[3]
    return [ 2*(X_var[0]-1) + 8*X_var[0]*(X_var[0]**2-X_var[1]) + X_var[2]*param[3],
            -4*(X_var[0]**2-X_var[1]) - X_var[2]*param[2],
            param[3]*(X_var[0]-param[0])-param[2]*(X_var[1]-param[1]) ]


# Gradient descente à pas optimal
#### Fonction pour faire plusieurs itérations
def gradient_descent_optimal(X, max_iterations):
    path = [X[:2]]  # stocker les valeurs à chaque iteration
    alpha = 0.01  # alpha initial

    for i in range(max_iterations):
        gradient = gradient_J3(X[:2])
        system_params = [X[0], X[1], gradient[0], gradient[1]]

        # résolution des équations
        sol = opt.root(systeme, X, system_params)
        X = sol.x

        # Modifie alpha
        alpha *= 0.9

        path.append(X[:2])

        #plt.scatter(X[0], X[1], label=f'Iteration {i}', color=f'C{i}')

    return path


###minimize
