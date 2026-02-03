def C_to_F(temp_C: float): #create function
    return temp_C* 1.8 + 32

tempc = 37
voltage = 14.5
system_name = "motor"

tempF = C_to_F(tempc)

safe_vol = 0<= voltage <= 20
safe_temp = 15<= tempc <= 80

system_safe =  safe_vol and safe_temp

status = "Ok, ready to arm" if system_safe else "False, not arm"

print(
      f"[{system_name}]\n"
      f"Temp: {tempF: .2f}\n"
      f"Voltage: {voltage: .1f}\n"
      f"[{status}]"
      
      )
