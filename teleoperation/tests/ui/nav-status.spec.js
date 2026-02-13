const { test, expect } = require('@playwright/test');

test('nav state panel visible', async ({ page }) => {
  await page.goto('/AutonTask');
  const panel = page.getByTestId('pw-nav-state-panel');
  await expect(panel).toBeVisible();
});

test('default state is OffState', async ({ page }) => {
  await page.goto('/AutonTask');
  const value = page.getByTestId('pw-nav-state-value');
  await expect(value).toBeVisible();
  await expect(value).toHaveText('OffState');
});

test('default color is error', async ({ page }) => {
  await page.goto('/AutonTask');
  const panel = page.getByTestId('pw-nav-state-panel');
  await expect(panel).toHaveClass(/nav-state--error/);
});

test('status panel visible', async ({ page }) => {
  await page.goto('/AutonTask');
  const statusPanel = page.getByTestId('pw-status-panel');
  await expect(statusPanel).toBeVisible();
});
