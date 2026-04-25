import asyncio
import logging

logger = logging.getLogger(__name__)


<<<<<<< HEAD
async def call_service_async(client, request, timeout=10.0):
    if not client.service_is_ready():
        logger.warning(f"Service {client.srv_name} not ready, skipping call")
        return None
=======
async def call_service_async(client, request, timeout=10.0) -> tuple:
    if not client.service_is_ready():
        reason = f"Service {client.srv_name} not ready"
        logger.warning(reason)
        return None, reason

    logger.info(f"Service {client.srv_name} is ready, sending request")
>>>>>>> origin/main

    loop = asyncio.get_running_loop()
    future = client.call_async(request)
    event = asyncio.Event()
    result = None
<<<<<<< HEAD

    def on_done(fut):
        nonlocal result
        try:
            result = fut.result()
        except Exception as exc:
            logger.error(f"Service call failed: {exc}")
=======
    error = None

    def on_done(fut):
        nonlocal result, error
        try:
            result = fut.result()
            logger.info(f"Service {client.srv_name} call completed successfully")
        except Exception as exc:
            error = f"Service call to {client.srv_name} failed: {exc}"
            logger.error(error)
>>>>>>> origin/main
        loop.call_soon_threadsafe(event.set)

    future.add_done_callback(on_done)

    try:
        await asyncio.wait_for(event.wait(), timeout=timeout)
<<<<<<< HEAD
        return result
    except asyncio.TimeoutError:
        future.cancel()
        return None
=======
        return result, error
    except asyncio.TimeoutError:
        logger.error(f"Service {client.srv_name} call timed out after {timeout}s")
        future.cancel()
        return None, f"Service {client.srv_name} timed out after {timeout}s"
>>>>>>> origin/main
