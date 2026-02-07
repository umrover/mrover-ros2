const { test, expect } = require('@playwright/test');

test.beforeEach(async ({ page }) => {
  await page.setViewportSize({ width: 1920, height: 1080 });
});

test('teleop toggle visible', async ({ page }) => {
  await page.goto('http://localhost:8080/AutonTask');
  await page.waitForLoadState('networkidle');
  const toggle = page.getByTestId('pw-teleop-toggle');
  await expect(toggle).toBeVisible({ timeout: 10000 });
});

test('teleop sends POST on click', async ({ page }) => {
  await page.goto('http://localhost:8080/AutonTask');
  await page.waitForLoadState('networkidle');

  const toggle = page.getByTestId('pw-teleop-toggle');
  await expect(toggle).toBeVisible({ timeout: 10000 });

  const enablePromise = page.waitForResponse(
    (resp) => resp.url().includes('/api/enable_teleop/') && resp.request().method() === 'POST'
  );
  await toggle.click();
  const response = await enablePromise;
  expect(response.status()).toBe(200);
  const body = await response.json();
  expect(body.status).toBe('success');
});

test('teleop shows enabled state', async ({ page }) => {
  await page.goto('http://localhost:8080/AutonTask');
  await page.waitForLoadState('networkidle');

  const toggle = page.getByTestId('pw-teleop-toggle');
  await expect(toggle).toBeVisible({ timeout: 10000 });

  const enablePromise = page.waitForResponse(
    (resp) => resp.url().includes('/api/enable_teleop/') && resp.request().method() === 'POST'
  );
  await toggle.click();
  await enablePromise;
  await expect(toggle).toHaveClass(/btn-success/, { timeout: 5000 });
});

test('teleop shows disabled after re-toggle', async ({ page }) => {
  await page.goto('http://localhost:8080/AutonTask');
  await page.waitForLoadState('networkidle');

  const toggle = page.getByTestId('pw-teleop-toggle');
  await expect(toggle).toBeVisible({ timeout: 10000 });

  // Enable
  let responsePromise = page.waitForResponse(
    (resp) => resp.url().includes('/api/enable_teleop/') && resp.request().method() === 'POST'
  );
  await toggle.click();
  await responsePromise;
  await expect(toggle).toHaveClass(/btn-success/, { timeout: 5000 });

  // Disable
  responsePromise = page.waitForResponse(
    (resp) => resp.url().includes('/api/enable_teleop/') && resp.request().method() === 'POST'
  );
  await toggle.click();
  const response = await responsePromise;
  const body = await response.json();
  expect(body.enabled).toBe(false);
  await expect(toggle).toHaveClass(/btn-danger/, { timeout: 5000 });
});

test('autonomy mode button visible', async ({ page }) => {
  await page.goto('http://localhost:8080/AutonTask');
  await page.waitForLoadState('networkidle');
  await expect(page.getByTestId('pw-auton-toggle')).toBeVisible({ timeout: 10000 });
});

test('pure pursuit button visible', async ({ page }) => {
  await page.goto('http://localhost:8080/AutonTask');
  await page.waitForLoadState('networkidle');
  await expect(page.getByTestId('pw-pure-pursuit-toggle')).toBeVisible({ timeout: 10000 });
});

test('path relaxation button visible', async ({ page }) => {
  await page.goto('http://localhost:8080/AutonTask');
  await page.waitForLoadState('networkidle');
  await expect(page.getByTestId('pw-path-relaxation-toggle')).toBeVisible({ timeout: 10000 });
});

test('path interpolation button visible', async ({ page }) => {
  await page.goto('http://localhost:8080/AutonTask');
  await page.waitForLoadState('networkidle');
  await expect(page.getByTestId('pw-path-interpolation-toggle')).toBeVisible({ timeout: 10000 });
});
