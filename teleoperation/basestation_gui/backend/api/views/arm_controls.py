from rest_framework.decorators import api_view
from rest_framework.response import Response
from rest_framework import status
from backend.ra_controls import set_rac_ra_mode

@api_view(['POST'])
def set_ra_mode(request):
    try:
        new_mode = request.data.get('mode')
        set_rac_ra_mode(new_mode)
        return Response({'status': 'success', 'mode': new_mode})
    except Exception as e:
        return Response({'status': 'error', 'message': str(e)},
                       status=status.HTTP_500_INTERNAL_SERVER_ERROR)