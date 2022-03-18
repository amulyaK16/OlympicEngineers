from django.db import models
from django.db.models.fields import DateTimeField
import string
import random
from django.core.validators import validate_comma_separated_integer_list

def generate_unique_code():
    length = 10

    while True:
        code = ''.join(random.choices(string.ascii_uppercase, k=length))
        if LiveData.objects.filter(uniqueCode=code).count() == 0:
            break

    return code

# Create your models here.
class User(models.Model):
    username = models.CharField(max_length=30, unique=True)
    password = models.CharField(max_length=20)
    email = models.EmailField(max_length=254)
    created_at = models.DateTimeField(auto_now_add=True)

class LiveData(models.Model):
    username = models.CharField(max_length=30, unique=True)
    uniqueCode = models.CharField(max_length=12, default=generate_unique_code, unique=True)
    ecg = models.CharField(validators=[validate_comma_separated_integer_list], max_length=1000)
    emg = models.CharField(validators=[validate_comma_separated_integer_list], max_length=1000)
    force = models.CharField(validators=[validate_comma_separated_integer_list], max_length=500)
    gyro_x = models.CharField(validators=[validate_comma_separated_integer_list], max_length=500)
    gyro_y = models.CharField(validators=[validate_comma_separated_integer_list], max_length=500)
    gyro_z = models.CharField(validators=[validate_comma_separated_integer_list], max_length=500)
    accel_x = models.CharField(validators=[validate_comma_separated_integer_list], max_length=500)
    accel_y = models.CharField(validators=[validate_comma_separated_integer_list], max_length=500)
    accel_z = models.CharField(validators=[validate_comma_separated_integer_list], max_length=500)
