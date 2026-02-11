const { test, expect } = require('@playwright/test');

test.beforeEach(async ({ page }) => {
  await page.setViewportSize({ width: 1920, height: 1080 });
});

test('teleop toggle button is visible on AutonTask page', async ({ page }) => {
  await page.goto('http://localhost:8080/AutonTask');
  await page.waitForLoadState('networkidle');

  const teleopToggle = page.getByTestId('pw-teleop-toggle');
  await expect(teleopToggle).toBeVisible({ timeout: 10000 });
});

test('clicking teleop toggle sends POST to /enable_teleop/', async ({ page }) => {
  await page.goto('http://localhost:8080/AutonTask');
  await page.waitForLoadState('networkidle');

  const teleopToggle = page.getByTestId('pw-teleop-toggle');
  await expect(teleopToggle).toBeVisible({ timeout: 10000 });

  const enablePromise = page.waitForResponse(
    (resp) => resp.url().includes('/api/enable_teleop/') && resp.request().method() === 'POST'
  );

  await teleopToggle.click();
  const response = await enablePromise;

  expect(response.status()).toBe(200);
  const body = await response.json();
  expect(body.status).toBe('success');
  expect(body.enabled).toBe(true);
});

test('teleop toggle shows enabled state after successful enable', async ({ page }) => {
  await page.goto('http://localhost:8080/AutonTask');
  await page.waitForLoadState('networkidle');

  const teleopToggle = page.getByTestId('pw-teleop-toggle');
  await expect(teleopToggle).toBeVisible({ timeout: 10000 });

  const enablePromise = page.waitForResponse(
    (resp) => resp.url().includes('/api/enable_teleop/') && resp.request().method() === 'POST'
  );

  await teleopToggle.click();
  await enablePromise;

  await expect(teleopToggle).toHaveClass(/btn-success/, { timeout: 5000 });
  const checkIcon = teleopToggle.locator('i.bi-check-square-fill');
  await expect(checkIcon).toBeVisible({ timeout: 5000 });
});

test('teleop toggle shows disabled state after disable', async ({ page }) => {
  await page.goto('http://localhost:8080/AutonTask');
  await page.waitForLoadState('networkidle');

  const teleopToggle = page.getByTestId('pw-teleop-toggle');
  await expect(teleopToggle).toBeVisible({ timeout: 10000 });

  const enablePromise = page.waitForResponse(
    (resp) => resp.url().includes('/api/enable_teleop/') && resp.request().method() === 'POST'
  );
  await teleopToggle.click();
  await enablePromise;

  await expect(teleopToggle).toHaveClass(/btn-success/, { timeout: 5000 });

  const disablePromise = page.waitForResponse(
    (resp) => resp.url().includes('/api/enable_teleop/') && resp.request().method() === 'POST'
  );
  await teleopToggle.click();
  const response = await disablePromise;

  expect(response.status()).toBe(200);
  const body = await response.json();
  expect(body.enabled).toBe(false);

  await expect(teleopToggle).toHaveClass(/btn-danger/, { timeout: 5000 });
  const uncheckedIcon = teleopToggle.locator('i.bi-square');
  await expect(uncheckedIcon).toBeVisible({ timeout: 5000 });
});
