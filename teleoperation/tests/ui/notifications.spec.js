const { test, expect } = require('@playwright/test');

test('bell button visible', async ({ page }) => {
  await page.goto('/');
  await expect(page.getByTestId('pw-notification-bell')).toBeVisible();
});

test('clicking bell opens panel', async ({ page }) => {
  await page.goto('/');
  const bell = page.getByTestId('pw-notification-bell');
  await expect(bell).toBeVisible();
  await bell.click();
  await expect(page.getByTestId('pw-notification-panel')).toBeVisible();
});

test('clicking outside closes panel', async ({ page }) => {
  await page.goto('/');
  const bell = page.getByTestId('pw-notification-bell');
  await expect(bell).toBeVisible();

  await bell.click();
  await expect(page.getByTestId('pw-notification-panel')).toBeVisible();

  const backdrop = page.locator('.fixed.top-0.left-0.w-full.h-full');
  await backdrop.click({ position: { x: 10, y: 10 } });
  await expect(page.getByTestId('pw-notification-panel')).not.toBeVisible({ timeout: 3000 });
});
