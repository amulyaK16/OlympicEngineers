from django.urls import path
from .views import *

urlpatterns = [
    path('', UserView.as_view()),
    path('CreateLiveSession', CreateLiveSession.as_view()),
    path('GetUserCode', GetUserCode.as_view()),
    path('SendData', SendData.as_view()),
    path('testapi', TestAPI.as_view())
]