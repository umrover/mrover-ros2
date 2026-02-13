const { test, expect } = require('@playwright/test');

test('frontend loads with title', async ({ page }) => {
  await page.goto('/');
  await expect(page).toHaveTitle(/MRover/);
});

test('4 mission buttons visible', async ({ page }) => {
  await page.goto('/');
  const buttons = page.getByTestId('pw-menu-btn');
  await expect(buttons).toHaveCount(4);
});

test('Delivery navigates to /DMTask', async ({ page }) => {
  await page.goto('/');
  const btn = page.getByTestId('pw-menu-btn').filter({ hasText: 'Delivery' });
  await expect(btn).toBeVisible();
  await btn.click();
  await expect(page).toHaveURL(/\/DMTask/);
});

test('Equipment Servicing navigates to /ESTask', async ({ page }) => {
  await page.goto('/');
  const btn = page.getByTestId('pw-menu-btn').filter({ hasText: 'Equipment Servicing' });
  await expect(btn).toBeVisible();
  await btn.click();
  await expect(page).toHaveURL(/\/ESTask/);
});

test('Science navigates to /ScienceTask', async ({ page }) => {
  await page.goto('/');
  const btn = page.getByTestId('pw-menu-btn').filter({ hasText: 'Science' });
  await expect(btn).toBeVisible();
  await btn.click();
  await expect(page).toHaveURL(/\/ScienceTask/);
});

test('Autonomy navigates to /AutonTask', async ({ page }) => {
  await page.goto('/');
  const btn = page.getByTestId('pw-menu-btn').filter({ hasText: 'Autonomy' });
  await expect(btn).toBeVisible();
  await btn.click();
  await expect(page).toHaveURL(/\/AutonTask/);
});
