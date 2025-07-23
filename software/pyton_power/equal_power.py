import numpy as np
import matplotlib.pyplot as plt

# Settings
STEP_PERCENT = 2  # Power step in percent (1, 2, 5, 10, etc.)
MAX_POWER = 100   # Maximum power (100%)
VALUES_PER_LINE = 5  # Number of values per line in array

def solve_theta(target, tol=1e-8):
    theta = (3 * target)**(1/3) if target < 0.1 else target
    for _ in range(100):
        f = theta - 0.5*np.sin(2*theta) - target
        df = 1 - np.cos(2*theta)
        if abs(df) < 1e-12:
            return solve_theta_bisection(target, tol)
        theta -= f / df
        if abs(f) < tol:
            return theta
    return solve_theta_bisection(target, tol)

def solve_theta_bisection(target, tol=1e-8):
    left, right = 0.0, np.pi
    while right - left > tol:
        mid = (left + right) / 2
        val = mid - 0.5*np.sin(2*mid) - target
        if val < 0:
            left = mid
        else:
            right = mid
    return (left + right) / 2

# =============================================
# 1. Generate and display table
# =============================================
steps = int(MAX_POWER / STEP_PERCENT)
power_percent = []
angles_deg = []

print("="*60)
print(f"Power to angle dependency table (step {STEP_PERCENT}%)")
print("="*60)
print("Power%\tAngle (°)\tF(θ)")
print("-"*40)
for k in range(steps + 1):
    target = k * np.pi / steps
    angle_rad = solve_theta(target) if k < steps else np.pi
    angle_deg = np.degrees(angle_rad)
    power = k * STEP_PERCENT
    
    power_percent.append(power)
    angles_deg.append(angle_deg)
    
    f_theta = angle_rad - 0.5*np.sin(2*angle_rad)
    print(f"{power}\t{angle_deg:.2f}\t\t{f_theta/np.pi:.3f}")



# =============================================
# 2. Generate array with comments
# =============================================
percent_values = [round(100 * (1 - angle/180)) for angle in angles_deg][::-1]
real_angles = [round(angle, 2) for angle in angles_deg][::-1]

print("\n" + "="*60)
print("// Percentage of 180° for equal power increments (step ~{}%)".format(STEP_PERCENT))
print("static const uint8_t phase_delay_percent[] =")
print("{")

for i in range(0, len(percent_values), VALUES_PER_LINE):
    values_line = percent_values[i:i+VALUES_PER_LINE]
    angles_line = real_angles[i:i+VALUES_PER_LINE]
    
    values_str = ", ".join([f"{100-val:3d}" for val in values_line])
    angles_str = ", ".join([f"{angle}°" for angle in angles_line])
    
    line = f"    {values_str.ljust(20)}  // {angles_str}"
    print(line)

print("};")
print("="*60)


# =============================================
# 3. Plot graph
# =============================================
plt.figure(figsize=(10, 6))
plt.plot(angles_deg, power_percent, 'bo-', linewidth=2, markersize=8)
plt.xlabel('Angle (degrees)', fontsize=12)
plt.ylabel('Power (%)', fontsize=12)
plt.title(f'Power vs. Angle Dependency (step {STEP_PERCENT}%)', fontsize=14)
plt.grid(True, linestyle='--', alpha=0.7)
plt.xticks(np.arange(0, 181, 15))
plt.yticks(np.arange(0, 101, 10))
plt.xlim(0, 180)
plt.ylim(0, 100)
plt.show()

