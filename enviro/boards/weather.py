import time, math, os
from breakout_bme280 import BreakoutBME280
from breakout_ltr559 import BreakoutLTR559
from machine import Pin, PWM
from pimoroni import Analog
from enviro import i2c, activity_led
import enviro.helpers as helpers
from phew import logging
from enviro.constants import WAKE_REASON_RTC_ALARM, WAKE_REASON_BUTTON_PRESS

# amount of rain required for the bucket to tip in mm
RAIN_MM_PER_TICK = 0.2794

# distance from the centre of the anemometer to the centre 
# of one of the cups in cm
WIND_CM_RADIUS = 7.0
# scaling factor for wind speed in m/s
WIND_FACTOR = 0.0218

bme280 = BreakoutBME280(i2c, 0x77)
ltr559 = BreakoutLTR559(i2c)

wind_direction_pin = Analog(26)
wind_speed_pin = Pin(9, Pin.IN, Pin.PULL_UP)
rain_pin = Pin(10, Pin.IN, Pin.PULL_DOWN)
last_rain_trigger = False

def startup(reason):
  global last_rain_trigger
  import wakeup

  # check if rain sensor triggered wake
  rain_sensor_trigger = wakeup.get_gpio_state() & (1 << 10)

  if rain_sensor_trigger:
    # read the current rain entries
    rain_entries = []
    if helpers.file_exists("rain.txt"):
      with open("rain.txt", "r") as rainfile:
        rain_entries = rainfile.read().split("\n")

    # add new entry
    logging.info(f"> add new rain trigger at {helpers.datetime_string()}")
    rain_entries.append(helpers.datetime_string())

    # limit number of entries to 190 - each entry is 21 bytes including
    # newline so this keeps the total rain.txt filesize just under one
    # filesystem block (4096 bytes)
    rain_entries = rain_entries[-190:]

    # write out adjusted rain log
    with open("rain.txt", "w") as rainfile:
      rainfile.write("\n".join(rain_entries))

    last_rain_trigger = True

    # if we were woken by the RTC or a Poke continue with the startup
    return (reason is WAKE_REASON_RTC_ALARM 
      or reason is WAKE_REASON_BUTTON_PRESS)

  # there was no rain trigger so continue with the startup
  return True

def check_trigger():
  global last_rain_trigger
  rain_sensor_trigger = rain_pin.value()

  if rain_sensor_trigger and not last_rain_trigger:
    activity_led(100)
    time.sleep(0.05)
    activity_led(0)

    # read the current rain entries
    rain_entries = []
    if helpers.file_exists("rain.txt"):
      with open("rain.txt", "r") as rainfile:
        rain_entries = rainfile.read().split("\n")

    # add new entry
    logging.info(f"> add new rain trigger at {helpers.datetime_string()}")
    rain_entries.append(helpers.datetime_string())

    # limit number of entries to 190 - each entry is 21 bytes including
    # newline so this keeps the total rain.txt filesize just under one 
    # filesystem block (4096 bytes)
    rain_entries = rain_entries[-190:]

    # write out adjusted rain log
    with open("rain.txt", "w") as rainfile:
      rainfile.write("\n".join(rain_entries))

  last_rain_trigger = rain_sensor_trigger

def wind_speed(sample_time_ms=10000):
  # get initial sensor state
  state = wind_speed_pin.value()

  # create an array for each sensor to log the times when the sensor state changed
  # then we can use those values to calculate an average tick time for each sensor
  ticks = []

  start = time.ticks_ms()
  while time.ticks_diff(time.ticks_ms(), start) <= sample_time_ms:
    now = wind_speed_pin.value()
    if now != state: # sensor output changed
      # record the time of the change and update the state
      ticks.append(time.ticks_ms())
      state = now

  logging.info(f"ticks: {ticks}")

  # Filter out duplicate ticks that may occur due to noise/bouncing
  filtered_ticks = []
  for tick in ticks:
    if not filtered_ticks or tick != filtered_ticks[-1]:
      filtered_ticks.append(tick)
  ticks = filtered_ticks

  logging.info(f"filtered_ticks: {filtered_ticks}")

  # if no sensor connected then we have no readings, skip
  if len(ticks) < 2:
    return 0, 0, 0

  # calculate the average, min and max tick between transitions in ms
  average_tick_ms = (time.ticks_diff(ticks[-1], ticks[0])) / (len(ticks) - 1)
  min_tick_ms = 10000
  max_tick_ms = 0
  for i in range(1, len(ticks)):
    tick_diff = time.ticks_diff(ticks[i], ticks[i-1])
    max_tick_ms = max(max_tick_ms, tick_diff)
    if max_tick_ms > 0:
      min_tick_ms_tmp = min(min_tick_ms, tick_diff)
      if min_tick_ms_tmp > 0:
        min_tick_ms = min_tick_ms_tmp

  logging.info(f"average_tick_ms: {average_tick_ms}")
  logging.info(f"min_tick_ms: {min_tick_ms}")
  logging.info(f"max_tick_ms: {max_tick_ms}")

  # Handle cases where individual values are 0
  avg_wind_m_s = min_wind_m_s = max_wind_m_s = 0
  
  if average_tick_ms == 0 and max_tick_ms == 0:
    return 0, 0, 0
    
  # Calculate only for non-zero values
  if average_tick_ms != 0:
    avg_rotation_hz = (1000 / average_tick_ms) / 2
    circumference = WIND_CM_RADIUS * 2.0 * math.pi
    avg_wind_m_s = avg_rotation_hz * circumference * WIND_FACTOR
    
  if min_tick_ms != 0:
    max_rotation_hz = (1000 / min_tick_ms) / 2
    circumference = WIND_CM_RADIUS * 2.0 * math.pi
    max_wind_m_s = max_rotation_hz * circumference * WIND_FACTOR
    
  if max_tick_ms != 0:
    min_rotation_hz = (1000 / max_tick_ms) / 2
    circumference = WIND_CM_RADIUS * 2.0 * math.pi
    min_wind_m_s = min_rotation_hz * circumference * WIND_FACTOR

  return avg_wind_m_s, max_wind_m_s, min_wind_m_s

def wind_direction():
    REFERENCE_VOLTAGE = 3.0  # Reference voltage that corresponds to 90 degrees
    
    # Read voltage with basic debouncing
    last_value = None
    while True:
        value = wind_direction_pin.read_voltage()
        
        if last_value is not None and abs(value - last_value) < 0.1:  # Debouncing threshold
            break
        last_value = value
    
    # Convert voltage to degrees: (voltage / 3.0V) * 90° to get the proper scaling
    degree = (value / REFERENCE_VOLTAGE) * 90
    
    # Ensure the value stays within 0-360 range
    degree = degree % 360
    
    return round(degree, 1)

def rainfall(seconds_since_last):
  amount = 0
  now = helpers.timestamp(helpers.datetime_string())
  if helpers.file_exists("rain.txt"):
    with open("rain.txt", "r") as rainfile:
      rain_entries = rainfile.read().split("\n")

    # count how many rain ticks since the last reading
    for entry in rain_entries:
      if entry:
        ts = helpers.timestamp(entry)
        if now - ts < seconds_since_last:
          amount += RAIN_MM_PER_TICK

    os.remove("rain.txt")
  
  per_second = 0
  if seconds_since_last > 0:
    per_second = amount / seconds_since_last

  return amount, per_second

def get_sensor_readings(seconds_since_last, is_usb_power):
  # bme280 returns the register contents immediately and then starts a new reading
  # we want the current reading so do a dummy read to discard register contents first
  bme280.read()
  time.sleep(0.1)
  bme280_data = bme280.read()

  ltr_data = ltr559.get_reading()
  rain, rain_per_second = rainfall(seconds_since_last)
  avg_wind_speed, max_wind_speed, min_wind_speed = wind_speed()

  from ucollections import OrderedDict
  return OrderedDict({
    "temperature": round(bme280_data[0], 2),
    "humidity": round(bme280_data[2], 2),
    "pressure": round(bme280_data[1] / 100.0, 2),
    "luminance": round(ltr_data[BreakoutLTR559.LUX], 2),
    "avg_wind_speed": avg_wind_speed,
    "min_wind_speed": min_wind_speed,
    "max_wind_speed": max_wind_speed,
    "rain": rain,
    "rain_per_second": rain_per_second,
    "wind_direction": wind_direction()
  })
