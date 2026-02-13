const { test, expect } = require('@playwright/test');

test('teleop toggle visible', async ({ page }) => {
  await page.goto('/AutonTask');
  const toggle = page.getByTestId('pw-teleop-toggle');
  await expect(toggle).toBeVisible();
});

test('teleop sends POST on click', async ({ page }) => {
  await page.goto('/AutonTask');

  const toggle = page.getByTestId('pw-teleop-toggle');
  await expect(toggle).toBeVisible();

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
  await page.goto('/AutonTask');

  const toggle = page.getByTestId('pw-teleop-toggle');
  await expect(toggle).toBeVisible();

  const enablePromise = page.waitForResponse(
    (resp) => resp.url().includes('/api/enable_teleop/') && resp.request().method() === 'POST'
  );
  await toggle.click();
  await enablePromise;
  await expect(toggle).toHaveClass(/btn-success/);
});

test('teleop shows disabled after re-toggle', async ({ page }) => {
  await page.goto('/AutonTask');

  const toggle = page.getByTestId('pw-teleop-toggle');
  await expect(toggle).toBeVisible();

  let responsePromise = page.waitForResponse(
    (resp) => resp.url().includes('/api/enable_teleop/') && resp.request().method() === 'POST'
  );
  await toggle.click();
  await responsePromise;
  await expect(toggle).toHaveClass(/btn-success/);

  responsePromise = page.waitForResponse(
    (resp) => resp.url().includes('/api/enable_teleop/') && resp.request().method() === 'POST'
  );
  await toggle.click();
  const response = await responsePromise;
  const body = await response.json();
  expect(body.enabled).toBe(false);
  await expect(toggle).toHaveClass(/btn-danger/);
});

test('autonomy mode button visible', async ({ page }) => {
  await page.goto('/AutonTask');
  await expect(page.getByTestId('pw-auton-toggle')).toBeVisible();
});

test('pure pursuit button visible', async ({ page }) => {
  await page.goto('/AutonTask');
  await expect(page.getByTestId('pw-pure-pursuit-toggle')).toBeVisible();
});

test('path relaxation button visible', async ({ page }) => {
  await page.goto('/AutonTask');
  await expect(page.getByTestId('pw-path-relaxation-toggle')).toBeVisible();
});

test('path interpolation button visible', async ({ page }) => {
  await page.goto('/AutonTask');
  await expect(page.getByTestId('pw-path-interpolation-toggle')).toBeVisible();
});
