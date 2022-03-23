from django.urls import path
from .views import *

urlpatterns = [
    path('', UserView.as_view()),
    path('CreateLiveSession', CreateLiveSession.as_view()),
    path('GetUserCode', GetUserCode.as_view()),
    path('SendData', SendData.as_view()),
    path('GetECG', GetECG.as_view()),
    path('GetEMG', GetEMG.as_view()),
    path('GetForce', GetForce.as_view()),
    path('PopData', PopData.as_view()),
    path('testapi', TestAPI.as_view())
]