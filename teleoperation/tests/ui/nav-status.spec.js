const { test, expect } = require('@playwright/test');

test.beforeEach(async ({ page }) => {
  await page.setViewportSize({ width: 1920, height: 1080 });
});

test('nav state panel visible', async ({ page }) => {
  await page.goto('http://localhost:8080/AutonTask');
  await page.waitForLoadState('networkidle');
  const panel = page.getByTestId('pw-nav-state-panel');
  await expect(panel).toBeVisible({ timeout: 10000 });
});

test('default state is OffState', async ({ page }) => {
  await page.goto('http://localhost:8080/AutonTask');
  await page.waitForLoadState('networkidle');
  await page.waitForTimeout(1000);
  const value = page.getByTestId('pw-nav-state-value');
  await expect(value).toBeVisible({ timeout: 10000 });
  await expect(value).toHaveText('OffState');
});

test('default color is error', async ({ page }) => {
  await page.goto('http://localhost:8080/AutonTask');
  await page.waitForLoadState('networkidle');
  await page.waitForTimeout(1000);
  const panel = page.getByTestId('pw-nav-state-panel');
  await expect(panel).toHaveClass(/nav-state--error/, { timeout: 5000 });
});

test('status panel visible', async ({ page }) => {
  await page.goto('http://localhost:8080/AutonTask');
  await page.waitForLoadState('networkidle');
  const statusPanel = page.getByTestId('pw-status-panel');
  await expect(statusPanel).toBeVisible({ timeout: 10000 });
});
