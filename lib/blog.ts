// Blog post types and utilities

export interface BlogPost {
  slug: string;
  title: string;
  description: string;
  date: string;
  author: {
    name: string;
    role?: string;
    avatar?: string;
  };
  category: 'Engineering' | 'Robotics' | 'AI' | 'Tutorial' | 'Announcement';
  tags: string[];
  readingTime: string;
  featured?: boolean;
  image?: string;
}

export interface BlogPostWithContent extends BlogPost {
  content: React.ReactNode;
}

// Calculate reading time from word count
export function calculateReadingTime(wordCount: number): string {
  const wordsPerMinute = 200;
  const minutes = Math.ceil(wordCount / wordsPerMinute);
  return `${minutes} min read`;
}

// Format date for display
export function formatDate(dateString: string): string {
  const date = new Date(dateString);
  return date.toLocaleDateString('en-US', {
    year: 'numeric',
    month: 'long',
    day: 'numeric',
  });
}

// Get category color
export function getCategoryColor(category: BlogPost['category']): string {
  const colors: Record<BlogPost['category'], string> = {
    Engineering: 'bg-blue-100 text-blue-800 dark:bg-blue-900 dark:text-blue-200',
    Robotics: 'bg-purple-100 text-purple-800 dark:bg-purple-900 dark:text-purple-200',
    AI: 'bg-green-100 text-green-800 dark:bg-green-900 dark:text-green-200',
    Tutorial: 'bg-orange-100 text-orange-800 dark:bg-orange-900 dark:text-orange-200',
    Announcement: 'bg-red-100 text-red-800 dark:bg-red-900 dark:text-red-200',
  };
  return colors[category];
}
