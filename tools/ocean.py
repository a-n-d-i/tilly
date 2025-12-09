#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from mpl_toolkits.mplot3d import Axes3D

# -------------------------------------------------------------
# JONSWAP Wave Spectrum (German Bight realistic parameters)
# -------------------------------------------------------------
def jonswap_spectrum(f, Hs=2.5, Tp=7.5, gamma=3.3):
    """
    Hs: Significant wave height (2.5 m typical moderate North Sea)
    Tp: Peak period (7.5 s typical German Bight)
    gamma: Peak enhancement factor (3.3 North Sea standard)
    """
    g = 9.81
    fp = 1.0 / Tp
    alpha = 0.076 * (Hs**2 * fp**4 / g**2)**0.22

    sigma = np.where(f <= fp, 0.07, 0.09)
    r = np.exp(- (f - fp)**2 / (2 * sigma**2 * fp**2))
    S = alpha * g**2 * (2 * np.pi)**(-4) * f**(-5) * np.exp(-1.25 * (fp / f)**4) * gamma**r
    return S

# -------------------------------------------------------------
# Create 2D wave field from spectrum
# -------------------------------------------------------------
def generate_wave_field(nx=100, ny=100, Lx=100, Ly=100):
    x = np.linspace(0, Lx, nx)
    y = np.linspace(0, Ly, ny)
    X, Y = np.meshgrid(x, y)
    return X, Y

# -------------------------------------------------------------
# Create waves from sum of components
# -------------------------------------------------------------
def generate_wave_components(N=200, Hs=2.5, Tp=7.5):
    f = np.linspace(0.05, 1.0, N)
    S = jonswap_spectrum(f, Hs, Tp)

    df = f[1] - f[0]
    amps = np.sqrt(2 * S * df)             # RMS amplitude from spectrum
    phases = np.random.uniform(0, 2*np.pi, N)
    directions = np.random.uniform(0, 2*np.pi, N)  # Short-crested

    return f, amps, phases, directions

# -------------------------------------------------------------
# Compute wave height at time t
# -------------------------------------------------------------
def wave_height(X, Y, t, f, amps, phases, directions):
    g = 9.81
    Z = np.zeros_like(X)

    for i in range(len(f)):
        k = (2*np.pi*f[i])**2 / g
        theta = directions[i]
        amp = amps[i]
        phase = phases[i]

        Z += amp * np.cos(
            k*(np.cos(theta)*X + np.sin(theta)*Y) - 2*np.pi*f[i]*t + phase
        )

    return Z

# -------------------------------------------------------------
# Main animation
# -------------------------------------------------------------
def animate_waves():
    X, Y = generate_wave_field()
    f, amps, phases, directions = generate_wave_components()

    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')

    Z = wave_height(X, Y, 0, f, amps, phases, directions)
    surf = ax.plot_surface(X, Y, Z, cmap='viridis')

    ax.set_zlim(-3, 3)
    ax.set_title("German Bight Wave Field (JONSWAP Spectrum)")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Height (m)")

    def update(frame):
        ax.clear()
        Z = wave_height(X, Y, frame * 0.1, f, amps, phases, directions)
        ax.plot_surface(X, Y, Z, cmap='viridis')
        ax.set_zlim(-3, 3)
        ax.set_title("German Bight Wave Field (JONSWAP Spectrum)")
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Height (m)")

    ani = animation.FuncAnimation(fig, update, frames=300, interval=30)
    plt.show()

# Run animation
if __name__ == "__main__":
    animate_waves()
