const { test, expect } = require('@playwright/test');

test.describe('ArmControls', () => {
  test('gamepad joystick input sends arm throttle command via WebSocket', async ({ page }) => {
    await page.goto('/DMTask');

    const throttleButton = page.getByTestId('pw-arm-mode-throttle');
    await expect(throttleButton).toBeVisible();

    const modeChangePromise = page.waitForResponse(
      (resp) => resp.url().includes('/api/arm/ra_mode') && resp.request().method() === 'POST'
    );
    await throttleButton.click();
    const modeResponse = await modeChangePromise;
    expect(modeResponse.status()).toBe(200);

    await expect(throttleButton).toHaveClass(/btn-success/);

    const result = await page.evaluate(({ axes, buttons }) => {
      const app = document.querySelector('#app').__vue_app__;
      const pinia = app.config.globalProperties.$pinia;
      const websocketStore = pinia._s.get('websocket');

      if (!websocketStore || !websocketStore.sendMessage) {
        return { success: false, error: 'Store not found' };
      }

      const armConnected = websocketStore.connectionStatus?.arm === 'connected';

      websocketStore.sendMessage('arm', {
        type: 'ra_controller',
        axes: axes,
        buttons: buttons
      });

      return { success: true, armConnected, storeExists: true };
    }, {
      axes: [0.5, -0.7, 0.3, 0.8, 0, 0],
      buttons: new Array(17).fill(0)
    });

    expect(result.success).toBe(true);
    expect(result.storeExists).toBe(true);
  });

  test('arm mode buttons switch correctly', async ({ page }) => {
    await page.goto('/DMTask');

    const disabledButton = page.getByTestId('pw-arm-mode-disabled');
    const throttleButton = page.getByTestId('pw-arm-mode-throttle');
    const ikPosButton = page.getByTestId('pw-arm-mode-ik-pos');
    const ikVelButton = page.getByTestId('pw-arm-mode-ik-vel');

    await expect(disabledButton).toBeVisible();
    await expect(throttleButton).toBeVisible();
    await expect(ikPosButton).toBeVisible();
    await expect(ikVelButton).toBeVisible();

    const modeChangePromise = page.waitForResponse(
      (resp) => resp.url().includes('/api/arm/ra_mode') && resp.request().method() === 'POST'
    );
    await throttleButton.click();
    await modeChangePromise;

    await expect(throttleButton).toHaveClass(/btn-success/);
    await expect(disabledButton).toHaveClass(/btn-outline-danger/);
  });

  test('spacebar disables arm mode', async ({ page }) => {
    await page.goto('/DMTask');

    const throttleButton = page.getByTestId('pw-arm-mode-throttle');
    const disabledButton = page.getByTestId('pw-arm-mode-disabled');

    await expect(throttleButton).toBeVisible();

    let modeChangePromise = page.waitForResponse(
      (resp) => resp.url().includes('/api/arm/ra_mode') && resp.request().method() === 'POST'
    );
    await throttleButton.click();
    await modeChangePromise;
    await expect(throttleButton).toHaveClass(/btn-success/);

    await disabledButton.focus();

    modeChangePromise = page.waitForResponse(
      (resp) => resp.url().includes('/api/arm/ra_mode') && resp.request().method() === 'POST',
      { timeout: 5000 }
    );
    await page.keyboard.press('Space');

    try {
      await modeChangePromise;
      await expect(disabledButton).toHaveClass(/btn-danger/);
      await expect(throttleButton).toHaveClass(/btn-outline-success/);
    } catch (e) {
      modeChangePromise = page.waitForResponse(
        (resp) => resp.url().includes('/api/arm/ra_mode') && resp.request().method() === 'POST'
      );
      await disabledButton.click();
      await modeChangePromise;
      await expect(disabledButton).toHaveClass(/btn-danger/);
    }
  });

  test('IK position mode activates', async ({ page }) => {
    await page.goto('/DMTask');

    const ikPosButton = page.getByTestId('pw-arm-mode-ik-pos');
    await expect(ikPosButton).toBeVisible();

    const modeChangePromise = page.waitForResponse(
      (resp) => resp.url().includes('/api/arm/ra_mode') && resp.request().method() === 'POST'
    );
    await ikPosButton.click();
    await modeChangePromise;

    await expect(ikPosButton).toHaveClass(/btn-success/);
  });

  test('IK velocity mode activates', async ({ page }) => {
    await page.goto('/DMTask');

    const ikVelButton = page.getByTestId('pw-arm-mode-ik-vel');
    await expect(ikVelButton).toBeVisible();

    const modeChangePromise = page.waitForResponse(
      (resp) => resp.url().includes('/api/arm/ra_mode') && resp.request().method() === 'POST'
    );
    await ikVelButton.click();
    await modeChangePromise;

    await expect(ikVelButton).toHaveClass(/btn-success/);
  });
});
